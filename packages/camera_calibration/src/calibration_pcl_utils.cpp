// cwru_pcl_utils: a  ROS library to illustrate use of PCL, including some handy utility functions

#include <camera_calibration/calibration_pcl_utils.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>

// uses initializer list for member vars

CalibrationPclUtils::CalibrationPclUtils(ros::NodeHandle* nodehandle) :   nh_(*nodehandle),
                                                            pclKinect_ptr_(new PointCloud<pcl::PointXYZ>),
                                                            pclKinect_clr_ptr_(new PointCloud<pcl::PointXYZRGB>),
                                                            pclTransformed_ptr_(new PointCloud<pcl::PointXYZ>),
                                                            pclTransformed_clr_ptr_(new PointCloud<pcl::PointXYZRGB>),
                                                            pclSelectedPoints_ptr_(new PointCloud<pcl::PointXYZ>),
                                                            pclTransformedSelectedPoints_ptr_(new PointCloud<pcl::PointXYZ>),
                                                            pclGenPurposeCloud_ptr_(new PointCloud<pcl::PointXYZ>),
                                                            pclGenPurposeCloud_clr_ptr_(new PointCloud<pcl::PointXYZRGB>),
                                                            planarCloud(new PointCloud<pcl::PointXYZ>),
                                                            blockCloud(new PointCloud<pcl::PointXYZRGB>)
{
    ROS_INFO("In constructor of CalibrationPclUtils");
    initializeSubscribers();
    initializePublishers();
    got_kinect_cloud_ = false;
    got_selected_points_ = false;
    ROS_INFO("Exiting constructor of CalibrationPclUtils");
}

void CalibrationPclUtils::fit_points_to_plane(Eigen::MatrixXf points_mat, Eigen::Vector3f &plane_normal, double &plane_dist)
{
    // ROS_INFO("starting identification of plane from data: ");
    int npts = points_mat.cols();  // number of points = number of columns in matrix; check the size
    
    // first compute the centroid of the data:
    // Eigen::Vector3f centroid; // make this member var, centroid_
    centroid_ = Eigen::MatrixXf::Zero(3, 1);  // see http://eigen.tuxfamily.org/dox/AsciiQuickReference.txt
    
    // centroid = compute_centroid(points_mat);
     for (int ipt = 0; ipt < npts; ipt++) {
        centroid_ += points_mat.col(ipt);  // add all the column vectors together
    }
    centroid_ /= npts;  // divide by the number of points to get the centroid    
    cout<<"centroid: "<<centroid_.transpose()<<endl;


    // subtract this centroid from all points in points_mat:
    Eigen::MatrixXf points_offset_mat = points_mat;
    for (int ipt = 0; ipt < npts; ipt++) {
        points_offset_mat.col(ipt) = points_offset_mat.col(ipt) - centroid_;
    }
    // compute the covariance matrix w/rt x,y,z:
    Eigen::Matrix3f CoVar;
    CoVar = points_offset_mat * (points_offset_mat.transpose()); // 3xN matrix times Nx3 matrix is 3x3
    // cout<<"covariance: "<<endl;
    // cout<<CoVar<<endl;

    // here is a more complex object: a solver for eigenvalues/eigenvectors;
    // we will initialize it with our covariance matrix, which will induce computing eval/evec pairs
    Eigen::EigenSolver<Eigen::Matrix3f> es3f(CoVar);

    Eigen::VectorXf evals; // we'll extract the eigenvalues to here
    // cout<<"size of evals: "<<es3d.eigenvalues().size()<<endl;
    // cout<<"rows,cols = "<<es3d.eigenvalues().rows()<<", "<<es3d.eigenvalues().cols()<<endl;
    // cout << "The eigenvalues of CoVar are:" << endl << es3d.eigenvalues().transpose() << endl;
    // cout<<"(these should be real numbers, and one of them should be zero)"<<endl;
    // cout << "The matrix of eigenvectors, V, is:" << endl;
    // cout<< es3d.eigenvectors() << endl << endl;
    // cout<< "(these should be real-valued vectors)"<<endl;
    // in general, the eigenvalues/eigenvectors can be complex numbers
    // however, since our matrix is self-adjoint (symmetric, positive semi-definite), we expect
    // real-valued evals/evecs;  we'll need to strip off the real parts of the solution

    evals = es3f.eigenvalues().real();  // grab just the real parts
    // cout<<"real parts of evals: "<<evals.transpose()<<endl;

    // our solution should correspond to an e-val of zero, which will be the minimum eval
    //  (all other evals for the covariance matrix will be >0)
    // however, the solution does not order the evals, so we'll have to find the one of interest ourselves

    double min_lambda = evals[0];  //initialize the hunt for min eval
    double max_lambda = evals[0];  // and for max eval
    // Eigen::Vector3cf complex_vec; // here is a 3x1 vector of double-precision, complex numbers
    // Eigen::Vector3f evec0, evec1, evec2; //, major_axis; 
    // evec0 = es3f.eigenvectors().col(0).real();
    //evec1 = es3f.eigenvectors().col(1).real();
    //evec2 = es3f.eigenvectors().col(2).real();  
    
    
    //((pt-centroid)*evec)*2 = evec'*points_offset_mat'*points_offset_mat*evec = 
    // = evec'*CoVar*evec = evec'*lambda*evec = lambda
    // min lambda is ideally zero for evec= plane_normal, since points_offset_mat*plane_normal~= 0
    // max lambda is associated with direction of major axis
    
    //sort the evals:
    
    //complex_vec = es3f.eigenvectors().col(0); // here's the first e-vec, corresponding to first e-val
    //cout<<"complex_vec: "<<endl;
    //cout<<complex_vec<<endl;
    plane_normal = es3f.eigenvectors().col(0).real(); //complex_vec.real(); //strip off the real part
    major_axis_ = es3f.eigenvectors().col(0).real(); // starting assumptions
    
    //cout<<"real part: "<<est_plane_normal.transpose()<<endl;
    //est_plane_normal = es3d.eigenvectors().col(0).real(); // evecs in columns

    double lambda_test;
    int i_normal = 0;
    int i_major_axis=0;
    //loop through "all" ("both", in this 3-D case) the rest of the solns, seeking min e-val
    for (int ivec = 1; ivec < 3; ivec++) {
        lambda_test = evals[ivec];
        if (lambda_test < min_lambda) {
            min_lambda = lambda_test;
            i_normal = ivec; //this index is closer to index of min eval
            plane_normal = es3f.eigenvectors().col(i_normal).real();
        }
        if (lambda_test > max_lambda) {
            max_lambda = lambda_test;
            i_major_axis = ivec; //this index is closer to index of min eval
            major_axis_ = es3f.eigenvectors().col(i_major_axis).real();
        }        
    }
    // at this point, we have the minimum eval in "min_lambda", and the plane normal
    // (corresponding evec) in "est_plane_normal"/
    // these correspond to the ith entry of i_normal
    cout<<"min eval is "<<min_lambda<<", corresponding to component "<<i_normal<<endl;
    cout<<"corresponding evec (est plane normal): "<<plane_normal.transpose()<<endl;
    cout<<"max eval is "<<max_lambda<<", corresponding to component "<<i_major_axis<<endl;
    cout<<"corresponding evec (est major axis): "<<major_axis_.transpose()<<endl;    
    
    //cout<<"correct answer is: "<<normal_vec.transpose()<<endl;
    plane_dist = plane_normal.dot(centroid_);
    //cout<<"est plane distance from origin = "<<est_dist<<endl;
    //cout<<"correct answer is: "<<dist<<endl;
    //cout<<endl<<endl;    

}

//get pts from cloud, pack the points into an Eigen::MatrixXf, then use above
// fit_points_to_plane fnc

void CalibrationPclUtils::fit_points_to_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr, Eigen::Vector3f &plane_normal, double &plane_dist) {
    Eigen::MatrixXf points_mat;
    Eigen::Vector3f cloud_pt;
    //populate points_mat from cloud data;

    int npts = input_cloud_ptr->points.size();
    points_mat.resize(3, npts);

    //somewhat odd notation: getVector3fMap() reading OR WRITING points from/to a pointcloud, with conversions to/from Eigen
    for (int i = 0; i < npts; ++i) {
        cloud_pt = input_cloud_ptr->points[i].getVector3fMap();
        points_mat.col(i) = cloud_pt;
    }
    fit_points_to_plane(points_mat, plane_normal, plane_dist);

}

//compute and return the centroid of a pointCloud
Eigen::Vector3f  CalibrationPclUtils::compute_centroid(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr) {
    Eigen::Vector3f centroid;
    Eigen::Vector3f cloud_pt;   
    int npts = input_cloud_ptr->points.size();    
    centroid<<0,0,0;
    //add all the points together:

    for (int ipt = 0; ipt < npts; ipt++) {
        cloud_pt = input_cloud_ptr->points[ipt].getVector3fMap();
        centroid += cloud_pt; //add all the column vectors together
    }
    centroid/= npts; //divide by the number of points to get the centroid
    return centroid;
}

//compute and return the centroid of a pointCloud
Eigen::Vector3f  CalibrationPclUtils::compute_centroid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr) {
    Eigen::Vector3f centroid;
    Eigen::Vector3f cloud_pt;   
    int npts = input_cloud_ptr->points.size();    
    centroid<<0,0,0;
    //add all the points together:

    for (int ipt = 0; ipt < npts; ipt++) {
        cloud_pt = input_cloud_ptr->points[ipt].getVector3fMap();
        centroid += cloud_pt; //add all the column vectors together
    }
    centroid/= npts; //divide by the number of points to get the centroid
    return centroid;
}

// this fnc operates on transformed selected points
void CalibrationPclUtils::fit_xformed_selected_pts_to_plane(Eigen::Vector3f &plane_normal, double &plane_dist) {
    fit_points_to_plane(pclTransformedSelectedPoints_ptr_, plane_normal, plane_dist);
    //Eigen::Vector3f centroid;
    cwru_msgs::PatchParams patch_params_msg;
    //compute the centroid; this is redundant w/ computation inside fit_points...oh well.
    // now the centroid computed by plane fit is stored in centroid_ member var
    //centroid = compute_centroid(pclTransformedSelectedPoints_ptr_);

    patch_params_msg.offset = plane_dist;
    patch_params_msg.centroid.resize(3);
    patch_params_msg.normal_vec.resize(3);
    for (int i=0;i<3;i++) {
        patch_params_msg.normal_vec[i]=plane_normal[i];
        patch_params_msg.centroid[i]= centroid_[i];
    }
    patch_params_msg.frame_id = "torso";
    patch_publisher_.publish(patch_params_msg);
}

Eigen::Affine3f CalibrationPclUtils::transformTFToEigen(const tf::Transform &t) {
    Eigen::Affine3f e;
    // treat the Eigen::Affine as a 4x4 matrix:
    for (int i = 0; i < 3; i++) {
        e.matrix()(i, 3) = t.getOrigin()[i]; //copy the origin from tf to Eigen
        for (int j = 0; j < 3; j++) {
            e.matrix()(i, j) = t.getBasis()[i][j]; //and copy 3x3 rotation matrix
        }
    }
    // Fill in identity in last row
    for (int col = 0; col < 3; col++)
        e.matrix()(3, col) = 0;
    e.matrix()(3, 3) = 1;
    return e;
}

/**here is a function that transforms a cloud of points into an alternative frame;
 * it assumes use of pclKinect_ptr_ from kinect sensor as input, to pclTransformed_ptr_ , the cloud in output frame
 * 
 * @param A [in] supply an Eigen::Affine3f, such that output_points = A*input_points
 */
void CalibrationPclUtils::transform_kinect_cloud(Eigen::Affine3f A) {
    transform_cloud(A, pclKinect_ptr_, pclTransformed_ptr_);
    transform_cloud(A, pclKinect_clr_ptr_, pclTransformed_clr_ptr_);
    /*
    pclTransformed_ptr_->header = pclKinect_ptr_->header;
    pclTransformed_ptr_->is_dense = pclKinect_ptr_->is_dense;
    pclTransformed_ptr_->width = pclKinect_ptr_->width;
    pclTransformed_ptr_->height = pclKinect_ptr_->height;
    int npts = pclKinect_ptr_->points.size();
    cout << "transforming npts = " << npts << endl;
    pclTransformed_ptr_->points.resize(npts);

    //somewhat odd notation: getVector3fMap() reading OR WRITING points from/to a pointcloud, with conversions to/from Eigen
    for (int i = 0; i < npts; ++i) {
        pclTransformed_ptr_->points[i].getVector3fMap() = A * pclKinect_ptr_->points[i].getVector3fMap(); 
    }    
     * */
}

void CalibrationPclUtils::transform_selected_points_cloud(Eigen::Affine3f A) {
    transform_cloud(A, pclSelectedPoints_ptr_, pclTransformedSelectedPoints_ptr_);
}

//    void get_transformed_selected_points(pcl::PointCloud<pcl::PointXYZ> & outputCloud );

void CalibrationPclUtils::get_transformed_selected_points(pcl::PointCloud<pcl::PointXYZ> & outputCloud ) {
    int npts = pclTransformedSelectedPoints_ptr_->points.size(); //how many points to extract?
    outputCloud.header = pclTransformedSelectedPoints_ptr_->header;
    outputCloud.is_dense = pclTransformedSelectedPoints_ptr_->is_dense;
    outputCloud.width = npts;
    outputCloud.height = 1;

    cout << "copying cloud w/ npts =" << npts << endl;
    outputCloud.points.resize(npts);
    for (int i = 0; i < npts; ++i) {
        outputCloud.points[i].getVector3fMap() = pclTransformedSelectedPoints_ptr_->points[i].getVector3fMap();   
    }
}

//same as above, but for general-purpose cloud
void CalibrationPclUtils::get_gen_purpose_cloud(pcl::PointCloud<pcl::PointXYZ> & outputCloud ) {
    int npts = pclGenPurposeCloud_ptr_->points.size(); //how many points to extract?
    outputCloud.header = pclGenPurposeCloud_ptr_->header;
    outputCloud.is_dense = pclGenPurposeCloud_ptr_->is_dense;
    outputCloud.width = npts;
    outputCloud.height = 1;

    cout << "copying cloud w/ npts =" << npts << endl;
    outputCloud.points.resize(npts);
    for (int i = 0; i < npts; ++i) {
        outputCloud.points[i].getVector3fMap() = pclGenPurposeCloud_ptr_->points[i].getVector3fMap();   
    }    
} 

void CalibrationPclUtils::get_gen_purpose_clr_cloud(pcl::PointCloud<pcl::PointXYZRGB> & outputCloud ) {
    int npts = pclGenPurposeCloud_ptr_->points.size(); //how many points to extract?
    outputCloud.header = pclGenPurposeCloud_ptr_->header;
    outputCloud.is_dense = pclGenPurposeCloud_ptr_->is_dense;
    outputCloud.width = npts;
    outputCloud.height = 1;

    cout << "copying cloud w/ npts =" << npts << endl;
    outputCloud.points.resize(npts);
    for (int i = 0; i < npts; ++i) {
        outputCloud.points[i].getVector3fMap() = pclGenPurposeCloud_clr_ptr_->points[i].getVector3fMap();   
        outputCloud.points[i].getRGBVector3i() = pclGenPurposeCloud_clr_ptr_->points[i].getRGBVector3i();   
    }    
}

//void CalibrationPclUtils::get_indices(vector<int> &indices,) {
//    indices = indicies_;
//}

//here is an example utility function.  It operates on clouds that are member variables, and it puts its result
// in the general-purpose cloud variable, which can be acquired by main(), if desired, using get_gen_purpose_cloud()

// The operation illustrated here is not all that useful.  It uses transformed, selected points,
// elevates the data by 5cm, and copies the result to the general-purpose cloud variable
void CalibrationPclUtils::example_pcl_operation() {
    int npts = pclTransformedSelectedPoints_ptr_->points.size(); //number of points
    copy_cloud(pclTransformedSelectedPoints_ptr_,pclGenPurposeCloud_ptr_); //now have a copy of the selected points in gen-purpose object
    Eigen::Vector3f offset;
    offset<<0,0,0.05;
    for (int i = 0; i < npts; ++i) {
        pclGenPurposeCloud_ptr_->points[i].getVector3fMap() = pclGenPurposeCloud_ptr_->points[i].getVector3fMap()+offset;   
    }    
} 

/**
 * @brief Finds coplanar points to the selected points cloud
 */
void CalibrationPclUtils::findCoplanarPoints() {
    double tolerance = 0;
    double min = std::numeric_limits<double>::max();
    double max = -9999999; // Should be std::numeric_limits<double>::lowest() if we had C++11 compatibility
   
    ROS_INFO("Min limit is %f", min);
    ROS_INFO("Max limit is %f", max);

    (*planarCloud).clear();
    (*pclGenPurposeCloud_ptr_).clear();
    // Loop through selected points and find the minimum and maximum Z coordinates
    if (got_selected_points_)
    {
        //ROS_INFO("Finding minimum and maximum height of selected points...");
        for (int i = 0; i  < pclTransformedSelectedPoints_ptr_->size(); i++)
        {
            if (pclTransformedSelectedPoints_ptr_->points[i].z < min)
            {
                min = pclTransformedSelectedPoints_ptr_->points[i].z;
            }

            if (pclTransformedSelectedPoints_ptr_->points[i].z > max)
            {
                    max = pclTransformedSelectedPoints_ptr_->points[i].z;
            }
        }
        
        ROS_INFO("Minimum z in selected cloud is %f", min);
        ROS_INFO("Maximum z in selected cloud is %f", max);
        ROS_INFO("Tolerance is %f", tolerance);
        min = min - tolerance;
        max = max + tolerance;
        //ROS_INFO("Minimum and maximum height of selected points found");
    }
    else 
    {
        ROS_WARN("Could not get minimum and maximum height because no points have been selected");
    }

    if (got_kinect_cloud_)
    {
        //ROS_INFO("Finding points in full cloud that are in the same plane as the selected points...");
        // Loop through full cloud and add points to plane cloud if they fall within the selected plane +- the tolerance
        for (int i = 0; i < pclTransformed_ptr_->size(); i++)
        {
            //ROS_INFO("Entered point finding loop iteration %d/%d", i, (int) pclKinect_ptr_->size());
            //ROS_INFO("Current point has coordinates (%f, %f, %f)", pclKinect_ptr_->points[i].x, pclKinect_ptr_->points[i].y, pclKinect_ptr_->points[i].z);
            if (pclTransformed_ptr_->points[i].z > min && pclTransformed_ptr_->points[i].z < max)
            {
                planarCloud->points.push_back(pclTransformed_ptr_->points[i]);
            }
        }

        //ROS_INFO("FInished created planar cloud");
    }
    else
    {
        ROS_WARN("Could not find planar points because no kinect cloud has been captured");
    }

    //ROS_INFO("Copying planar cloud to general purpose cloud");
    copy_cloud(planarCloud, pclGenPurposeCloud_ptr_);
    //ROS_INFO("Finish cloud copying");
} 

/**
 * @brief Finds the centroid of a selected patch of points
 *
 *  pcl::removeNaNFromPointCloud(pclKinect_clr_ptr_, pclKinect_clr_ptr_, index);
 * @param Pose A pose to have the centroid coordinates stored in
 */
void CalibrationPclUtils::find_selected_centroid(geometry_msgs::PoseStamped& Pose)
{

    if (got_selected_points_)
    {
        Eigen::Vector3f centroid = compute_centroid(pclTransformedSelectedPoints_ptr_);

        Pose.pose.position.x = centroid(0);
        Pose.pose.position.y = centroid(1);
        Pose.pose.position.z = centroid(2);

        ROS_INFO("Calculated centroid at (%f, %f, %f)", centroid(0), centroid(1), centroid(2));
    }
    else
    {
        ROS_WARN("No points selected, could not calculate centroid, returned pose unchanged");
    }
}

void CalibrationPclUtils::find_block(Eigen::Vector3f& centroid, Eigen::Vector3f& orientation, Eigen::Vector3d& color)
{
    ROS_INFO("Finding block...");
    if (got_kinect_cloud_)
    {
        ROS_INFO("Got kinect cloud");
        save_transformed_kinect_snapshot();
        save_kinect_snapshot();
        //std::vector<int> index;
        //copy_cloud(pclTransformed_clr_ptr_, kinectCloud);
        //pcl::removeNaNFromPointCloud(kinectCloud, kinectCloud, index);
        //ROS_INFO_STREAM("Size of cloud after NaN filtering is " << kinectCloud.size()); 
        double tableHeight = -.24; //  Need to determine this experimentally
        double maxHeight = 0;  // Need to set this based on height of blocks we're looking for
        double blockHeight = -.2;
        double heightErrorMargin = .015;

        Eigen::Vector3i blockColor; 
        for (int i = 0; i < pclTransformed_clr_ptr_->size(); i++)
        {
            if (pclTransformed_clr_ptr_->points[i].z > blockHeight)  // Is the point higher than where we think the current block top is
            {
                //ROS_INFO_STREAM("Point with height " << pclTransformed_clr_ptr_->points[i].z);
                if (pclTransformed_clr_ptr_->points[i].z < maxHeight)  // If so, is it lower than our expected max height?
                {
                    //ROS_INFO("A point passed the max height test");
                    blockColor = pclTransformed_clr_ptr_->points[i].getRGBVector3i();
                    if (blockColor(0) - 166 > 1 && blockColor(1) - 155 > 2 && blockColor(2) - 155 > 2)
                    {
                        //ROS_INFO("A point passed the color test");
                        blockHeight = pclTransformed_clr_ptr_->points[i].z;  // If yes then this is our new suggested block height     
                    }
                }
            } 
        }
        
        ROS_INFO_STREAM("Block height is " << blockHeight); 
        double max = blockHeight + heightErrorMargin;
        double min = blockHeight - heightErrorMargin;
        
        ROS_INFO("Starting second for loop");
        for (int i =0; i < kinectCloud.size(); i++)
        {
            if (kinectCloud.points[i].z >= min && kinectCloud.points[i].z <= max)
            {

                blockColor = kinectCloud.points[i].getRGBVector3i();
                if (blockColor(0) - 166 > 1 && blockColor(1) - 155 > 2 && blockColor(2) - 155 > 2)
                {
                    blockCloud->points.push_back(kinectCloud.points[i]);
                }
            }
        }

        copy_cloud(blockCloud, pclGenPurposeCloud_clr_ptr_);

        centroid = compute_centroid(blockCloud);
        ROS_INFO_STREAM("Computed centroid at\n" << centroid);

        Eigen::Vector3f left;  // +y
            left << 0,0,0;
        Eigen::Vector3f right; // -y
            right << 0,0,0;
        Eigen::Vector3f far;  // +x
            far << 0,0,0; 
        Eigen::Vector3f near;  // -x
            near << 0,0,0;
        double orientationErrorMargin = 0;

        // Find leftmost, farthest, and nearest point of the block
        /*for (int i = 0; i < kinectCloud.size(); i++)
        {
            if (kinectCloud.points[i].x > far(0))
            {
                far(0) = kinectCloud.points[i].x;
                far(1) = kinectCloud.points[i].y;
                far(2) = >points[i].z;
            }
            if (pclTransformed_clr_ptr_->points[i].x < near(0))
            {
                near(0) = pclTransformed_clr_ptr_->points[i].x;
                near(1) = pclTransformed_clr_ptr_->points[i].y;
                near(2) = pclTransformed_clr_ptr_->points[i].z;
            }
            if (pclTransformed_clr_ptr_->points[i].y > left(1))
            {
                left(0) = pclTransformed_clr_ptr_->points[i].x;
                left(1) = pclTransformed_clr_ptr_->points[i].y;
                left(2) = pclTransformed_clr_ptr_->points[i].z;
            }
            if (pclTransformed_clr_ptr_->points[i].y < right(1))
            {
                right(0) = pclTransformed_clr_ptr_->points[i].x;
                right(1) = pclTransformed_clr_ptr_->points[i].y;
                right(2) = pclTransformed_clr_ptr_->points[i].z;
            }
        }*/
        
        if (abs(far(1) - near(1)) < orientationErrorMargin)  // Block is reasonably straight
        {
            orientation = left - right; 
        }
        else
        {
            Eigen::Vector3f side1 = left - far;
            Eigen::Vector3f side2 = left - near;

            if (side1.norm() < side2.norm()) // If side1 is the long side
            {
                orientation =  side1;
            }
            else
            {
                orientation =  side2;
            }
        }
        ROS_INFO_STREAM("Computed orientation is\n" << orientation);

        // Now get color
        color = find_avg_color();
        ROS_INFO_STREAM("Computed average color is\n" << color);
    }
    else {
        ROS_INFO("No cloud available");
    }
}
//This fnc populates and output cloud of type XYZRGB extracted from the full Kinect cloud (in Kinect frame)
// provide a vector of indices and a holder for the output cloud, which gets populated
void CalibrationPclUtils::copy_indexed_pts_to_output_cloud(vector<int> &indices,PointCloud<pcl::PointXYZRGB> &outputCloud) {
    int npts = indices.size(); //how many points to extract?
    outputCloud.header = pclKinect_clr_ptr_->header;
    outputCloud.is_dense = pclKinect_clr_ptr_->is_dense;
    outputCloud.width = npts;
    outputCloud.height = 1;
    int i_index;

    cout << "copying cloud w/ npts =" << npts << endl;
    outputCloud.points.resize(npts);
    for (int i = 0; i < npts; ++i) {
        i_index = indices[i];
        outputCloud.points[i].getVector3fMap() = pclKinect_clr_ptr_->points[i_index].getVector3fMap();
        outputCloud.points[i].r = pclKinect_clr_ptr_->points[i_index].r;
        outputCloud.points[i].g = pclKinect_clr_ptr_->points[i_index].g;
        outputCloud.points[i].b = pclKinect_clr_ptr_->points[i_index].b;
        /*
            std::cout <<i_index
              << "    " << (int) pclKinect_clr_ptr_->points[i_index].r
              << " "    << (int) pclKinect_clr_ptr_->points[i_index].g
              << " "    << (int) pclKinect_clr_ptr_->points[i_index].b << std::endl;
         */
    }
} 


// comb through kinect colors and compute average color
// disregard color=0,0,0 
Eigen::Vector3d CalibrationPclUtils::find_avg_color() {
    Eigen::Vector3d avg_color;
    Eigen::Vector3d pt_color;
    Eigen::Vector3d ref_color;
    indices_.clear();
    ref_color<<147,147,147;
    int npts = pclKinect_clr_ptr_->points.size();
    int npts_colored = 0;
    for (int i=0;i<npts;i++) {
        pt_color(0) = (double) pclKinect_clr_ptr_->points[i].r;
        pt_color(1) = (double) pclKinect_clr_ptr_->points[i].g;
        pt_color(2) = (double) pclKinect_clr_ptr_->points[i].b;

    if ((pt_color-ref_color).norm() > 1) {
        avg_color+= pt_color;
        npts_colored++;
        indices_.push_back(i); // save this points as "interesting" color
    }
    }
    ROS_INFO("found %d points with interesting color",npts_colored);
    avg_color/=npts_colored;
    ROS_INFO("avg interesting color = %f, %f, %f",avg_color(0),avg_color(1),avg_color(2));
    return avg_color;
 
}

Eigen::Vector3d CalibrationPclUtils::find_avg_color_selected_pts(vector<int> &indices) {
    Eigen::Vector3d avg_color;
    Eigen::Vector3d pt_color;
    //Eigen::Vector3d ref_color;

    int npts = indices.size();
    int index;

    for (int i=0;i<npts;i++) {
        index = indices[i];
        pt_color(0) = (double) pclKinect_clr_ptr_->points[index].r;
        pt_color(1) = (double) pclKinect_clr_ptr_->points[index].g;
        pt_color(2) = (double) pclKinect_clr_ptr_->points[index].b;
        avg_color+= pt_color;
    }
    avg_color/=npts;
    ROS_INFO("avg color = %f, %f, %f",avg_color(0),avg_color(1),avg_color(2));
    return avg_color;
}

void CalibrationPclUtils::find_indices_color_match(vector<int> &input_indices,
                    Eigen::Vector3d normalized_avg_color,
                    double color_match_thresh, vector<int> &output_indices) {
     Eigen::Vector3d pt_color;

    int npts = input_indices.size();
    output_indices.clear();
    int index;
    int npts_matching = 0;

    for (int i=0;i<npts;i++) {
        index = input_indices[i];
        pt_color(0) = (double) pclKinect_clr_ptr_->points[index].r;
        pt_color(1) = (double) pclKinect_clr_ptr_->points[index].g;
        pt_color(2) = (double) pclKinect_clr_ptr_->points[index].b;
        pt_color = pt_color/pt_color.norm(); //compute normalized color
        if ((normalized_avg_color-pt_color).norm()<color_match_thresh) {
            output_indices.push_back(index);  //color match, so save this point index
            npts_matching++;
        }
    }   
    ROS_INFO("found %d color-match points from indexed set",npts_matching);
    
} 

//special case of above for transformed Kinect pointcloud:
void CalibrationPclUtils::filter_cloud_z(double z_nom, double z_eps, 
                double radius, Eigen::Vector3f centroid, vector<int> &indices) {
   filter_cloud_z(pclTransformed_ptr_, z_nom, z_eps, radius, centroid,indices);      
}

//operate on transformed Kinect pointcloud:
void CalibrationPclUtils::find_coplanar_pts_z_height(double plane_height,double z_eps,vector<int> &indices) {
    filter_cloud_z(pclTransformed_ptr_,plane_height,z_eps,indices);
}

void CalibrationPclUtils::filter_cloud_z(PointCloud<pcl::PointXYZ>::Ptr inputCloud, double z_nom, double z_eps, vector<int> &indices) {
    int npts = inputCloud->points.size();
    Eigen::Vector3f pt;
    indices.clear();
    double dz;
    int ans;
    for (int i = 0; i < npts; ++i) {
        pt = inputCloud->points[i].getVector3fMap();
        //cout<<"pt: "<<pt.transpose()<<endl;
        dz = pt[2] - z_nom;
        if (fabs(dz) < z_eps) {
            indices.push_back(i);
            //cout<<"dz = "<<dz<<"; saving this point...enter 1 to continue: ";
            //cin>>ans;
        }
    }
    int n_extracted = indices.size();
    cout << " number of points in range = " << n_extracted << endl;
}

//find points that are both (approx) coplanar at height z_nom AND within "radius" of "centroid"
void CalibrationPclUtils::filter_cloud_z(PointCloud<pcl::PointXYZ>::Ptr inputCloud, double z_nom, double z_eps, 
                double radius, Eigen::Vector3f centroid, vector<int> &indices)  {
    int npts = inputCloud->points.size();
    Eigen::Vector3f pt;
    indices.clear();
    double dz;
    int ans;
    for (int i = 0; i < npts; ++i) {
        pt = inputCloud->points[i].getVector3fMap();
        //cout<<"pt: "<<pt.transpose()<<endl;
        dz = pt[2] - z_nom;
        if (fabs(dz) < z_eps) {
            //passed z-test; do radius test:
            if ((pt-centroid).norm()<radius) {
               indices.push_back(i);
            }
            //cout<<"dz = "<<dz<<"; saving this point...enter 1 to continue: ";
            //cin>>ans;
        }
    }
    int n_extracted = indices.size();
    cout << " number of points in range = " << n_extracted << endl;    
    
}
    

void CalibrationPclUtils::analyze_selected_points_color() {
    int npts = pclTransformedSelectedPoints_ptr_->points.size(); //number of points
    //copy_cloud(pclTransformedSelectedPoints_ptr_,pclGenPurposeCloud_ptr_); //now have a copy of the selected points in gen-purpose object
    //Eigen::Vector3f offset;
    //offset<<0,0,0.05;
    int npts_clr = pclSelectedPtsClr_ptr_->points.size();
    cout<<"color pts size = "<<npts_clr<<endl;
        pcl::PointXYZRGB p;
        // unpack rgb into r/g/b
        uint32_t rgb = *reinterpret_cast<int*>(&p.rgb);
        uint8_t r,g,b;
        int r_int;
    
    for (int i = 0; i < npts; ++i) {
        p = pclSelectedPtsClr_ptr_->points[i];
        r = (rgb >> 16) & 0x0000ff;
        r_int = (int) r;
        // g = (rgb >> 8)  & 0x0000ff;
        // b = (rgb)       & 0x0000ff;
        cout<<"r_int: "<<r_int<<endl;
        cout<<"r1: "<<r<<endl;
        r=pclSelectedPtsClr_ptr_->points[i].r;
        cout<<"r2 = "<<r<<endl;
 
        //cout<<" ipt, r,g,b = "<<i<<","<<pclSelectedPtsClr_ptr_->points[i].r<<", "<<
        //        pclSelectedPtsClr_ptr_->points[i].g<<", "<<pclSelectedPtsClr_ptr_->points[i].b<<endl;
        //pclGenPurposeCloud_ptr_->points[i].getVector3fMap() = pclGenPurposeCloud_ptr_->points[i].getVector3fMap()+offset;   
    }    
        cout<<"done combing through selected pts"<<endl;
        got_kinect_cloud_=false; // get a new snapshot
}

//generic function to copy an input cloud to an output cloud
// provide pointers to the two clouds
//output cloud will get resized
void CalibrationPclUtils::copy_cloud(PointCloud<pcl::PointXYZ>::Ptr inputCloud, PointCloud<pcl::PointXYZ>::Ptr outputCloud) {
    int npts = inputCloud->points.size(); //how many points to extract?
    outputCloud->header = inputCloud->header;
    outputCloud->is_dense = inputCloud->is_dense;
    outputCloud->width = npts;
    outputCloud->height = 1;

    cout << "copying cloud w/ npts =" << npts << endl;
    outputCloud->points.resize(npts);
    for (int i = 0; i < npts; ++i) {
        outputCloud->points[i].getVector3fMap() = inputCloud->points[i].getVector3fMap();
    }
}

void CalibrationPclUtils::copy_cloud(PointCloud<pcl::PointXYZRGB>::Ptr inputCloud, PointCloud<pcl::PointXYZRGB>::Ptr outputCloud) {
    int npts = inputCloud->points.size(); //how many points to extract?
    outputCloud->header = inputCloud->header;
    outputCloud->is_dense = inputCloud->is_dense;
    outputCloud->width = npts;
    outputCloud->height = 1;

    cout << "copying cloud w/ npts =" << npts << endl;
    outputCloud->points.resize(npts);
    for (int i = 0; i < npts; ++i) {
        outputCloud->points[i].getVector3fMap() = inputCloud->points[i].getVector3fMap();
        outputCloud->points[i].getRGBVector3i() = inputCloud->points[i].getRGBVector3i();
    }
}

void CalibrationPclUtils::copy_cloud(PointCloud<pcl::PointXYZRGB>::Ptr inputCloud, PointCloud<pcl::PointXYZRGB>& outputCloud) {
    int npts = inputCloud->points.size(); //how many points to extract?
    outputCloud.header = inputCloud->header;
    outputCloud.is_dense = inputCloud->is_dense;
    outputCloud.width = npts;
    outputCloud.height = 1;

    cout << "copying cloud w/ npts =" << npts << endl;
    outputCloud.points.resize(npts);
    for (int i = 0; i < npts; ++i) {
        outputCloud.points[i].getVector3fMap() = inputCloud->points[i].getVector3fMap();
        outputCloud.points[i].getRGBVector3i() = inputCloud->points[i].getRGBVector3i();
    }
}
//given indices of interest, chosen points from input colored cloud to output colored cloud
void CalibrationPclUtils::copy_cloud_xyzrgb_indices(PointCloud<pcl::PointXYZRGB>::Ptr inputCloud, vector<int> &indices, PointCloud<pcl::PointXYZRGB>::Ptr outputCloud) {
    int npts = indices.size(); //how many points to extract?
    outputCloud->header = inputCloud->header;
    outputCloud->is_dense = inputCloud->is_dense;
    outputCloud->width = npts;
    outputCloud->height = 1;

    cout << "copying cloud w/ npts =" << npts << endl;
    outputCloud->points.resize(npts);
    for (int i = 0; i < npts; ++i) {
        outputCloud->points[i].getVector3fMap() = inputCloud->points[indices[i]].getVector3fMap();
    }
}



//need to fix this to put proper frame_id in header

void CalibrationPclUtils::transform_cloud(Eigen::Affine3f A, pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr,
        pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_ptr) {
    output_cloud_ptr->header = input_cloud_ptr->header;
    output_cloud_ptr->is_dense = input_cloud_ptr->is_dense;
    output_cloud_ptr->width = input_cloud_ptr->width;
    output_cloud_ptr->height = input_cloud_ptr->height;
    int npts = input_cloud_ptr->points.size();
    cout << "transforming npts = " << npts << endl;
    output_cloud_ptr->points.resize(npts);

    //somewhat odd notation: getVector3fMap() reading OR WRITING points from/to a pointcloud, with conversions to/from Eigen
    for (int i = 0; i < npts; ++i) {
        output_cloud_ptr->points[i].getVector3fMap() = A * input_cloud_ptr->points[i].getVector3fMap();
    }
}

void CalibrationPclUtils::transform_cloud(Eigen::Affine3f A, pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud_ptr) {
    output_cloud_ptr->header = input_cloud_ptr->header;
    output_cloud_ptr->is_dense = input_cloud_ptr->is_dense;
    output_cloud_ptr->width = input_cloud_ptr->width;
    output_cloud_ptr->height = input_cloud_ptr->height;
    int npts = input_cloud_ptr->points.size();
    cout << "transforming npts = " << npts << endl;

    output_cloud_ptr->points.resize(npts);
    ROS_INFO("Resized output_cloud");
    //somewhat odd notation: getVector3fMap() reading OR WRITING points from/to a pointcloud, with conversions to/from Eigen
    for (int i = 0; i < npts; ++i) {
        output_cloud_ptr->points[i].getVector3fMap() = A * input_cloud_ptr->points[i].getVector3fMap();
        output_cloud_ptr->points[i].getRGBVector3i() = input_cloud_ptr->points[i].getRGBVector3i();
    }
}

//member helper function to set up subscribers;
// note odd syntax: &ExampleRosClass::subscriberCallback is a pointer to a member function of ExampleRosClass
// "this" keyword is required, to refer to the current instance of ExampleRosClass

void CalibrationPclUtils::initializeSubscribers() {
    ROS_INFO("Initializing Subscribers");

    pointcloud_subscriber_ = nh_.subscribe("/kinect/depth/points", 1, &CalibrationPclUtils::kinectCB, this);
    // add more subscribers here, as needed

    // subscribe to "selected_points", which is published by Rviz tool
    selected_points_subscriber_ = nh_.subscribe<sensor_msgs::PointCloud2> ("/selected_points", 1, &CalibrationPclUtils::selectCB, this);
}

//member helper function to set up publishers;

void CalibrationPclUtils::initializePublishers() {
    ROS_INFO("Initializing Publishers");
    pointcloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("cwru_pcl_pointcloud", 1, true);
    patch_publisher_ = nh_.advertise<cwru_msgs::PatchParams>("pcl_patch_params", 1, true);
    //add more publishers, as needed
    // note: COULD make minimal_publisher_ a public member function, if want to use it within "main()"
}

/**
 * callback fnc: receives transmissions of Kinect data; if got_kinect_cloud is false, copy current transmission to internal variable
 * @param cloud [in] messages received from Kinect
 */
void CalibrationPclUtils::kinectCB(const sensor_msgs::PointCloud2ConstPtr& cloud) {
    //cout<<"callback from kinect pointcloud pub"<<endl;
    // convert/copy the cloud only if desired
    if (!got_kinect_cloud_) {
        pcl::fromROSMsg(*cloud, *pclKinect_ptr_);
        pcl::fromROSMsg(*cloud,*pclKinect_clr_ptr_);
        ROS_INFO("kinectCB: got cloud with %d * %d points", (int) pclKinect_ptr_->width, (int) pclKinect_ptr_->height);
        got_kinect_cloud_ = true; //cue to "main" that callback received and saved a pointcloud 
        //check some colors:
   int npts_clr = pclKinect_clr_ptr_->points.size();
    cout<<"Kinect color pts size = "<<npts_clr<<endl;
    avg_color_ = find_avg_color();
    /*
     for (size_t i = 0; i < pclKinect_clr_ptr_->points.size (); ++i)
     std::cout << " " << (int) pclKinect_clr_ptr_->points[i].r
              << " "    << (int) pclKinect_clr_ptr_->points[i].g
              << " "    << (int) pclKinect_clr_ptr_->points[i].b << std::endl;   

 
        //cout<<" ipt, r,g,b = "<<i<<","<<pclSelectedPtsClr_ptr_->points[i].r<<", "<<
        //        pclSelectedPtsClr_ptr_->points[i].g<<", "<<pclSelectedPtsClr_ptr_->points[i].b<<endl;
        //pclGenPurposeCloud_ptr_->points[i].getVector3fMap() = pclGenPurposeCloud_ptr_->points[i].getVector3fMap()+offset;   

        cout<<"done combing through selected pts"<<endl;   
     *     */     
    }
    //pcl::io::savePCDFileASCII ("snapshot.pcd", *g_pclKinect);
    //ROS_INFO("saved PCD image consisting of %d data points to snapshot.pcd",(int) g_pclKinect->points.size ()); 
}

// this callback wakes up when a new "selected Points" message arrives
void CalibrationPclUtils::selectCB(const sensor_msgs::PointCloud2ConstPtr& cloud) {
    pcl::fromROSMsg(*cloud, *pclSelectedPoints_ptr_);
    
    //looks like selected points does NOT include color of points
    //pcl::fromROSMsg(*cloud, *pclSelectedPtsClr_ptr_); //color version  

    ROS_INFO("RECEIVED NEW PATCH w/  %d * %d points", pclSelectedPoints_ptr_->width, pclSelectedPoints_ptr_->height);
 
    /*
    ROS_INFO("Color version has  %d * %d points", pclSelectedPtsClr_ptr_->width, pclSelectedPtsClr_ptr_->height);

    for (size_t i = 0; i < pclSelectedPtsClr_ptr_->points.size (); ++i) {
    std::cout <<i<<": "
              << "    " << (int) pclSelectedPtsClr_ptr_->points[i].r
              << " "    << (int) pclSelectedPtsClr_ptr_->points[i].g
              << " "    << (int) pclSelectedPtsClr_ptr_->points[i].b << std::endl;
    }
     * */
    ROS_INFO("done w/ selected-points callback");

    got_selected_points_ = true;
}

geometry_msgs::PoseStamped CalibrationPclUtils::eigenToPose(Eigen::Vector3f& eigen)
{
    geometry_msgs::PoseStamped pose;

    ROS_INFO_STREAM("Recieved Eigen::Vector3f with value " << 
        eigen(0) << "," <<
        eigen(1) << "," <<
        eigen(2));    
    pose.pose.position.x = eigen(0);
    pose.pose.position.y = eigen(1);
    pose.pose.position.z = eigen(2);
    
    ROS_INFO_STREAM("Returning pose with origin: " << pose.pose.position.x << "," <<
        pose.pose.position.y << "," <<
        pose.pose.position.z);
    return pose;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr CalibrationPclUtils::getBlockCloud()
{
        return blockCloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr CalibrationPclUtils::getKinectColorCloud()
{
    return pclKinect_clr_ptr_;
}

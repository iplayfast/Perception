#!/usr/bin/env python

# Import modules
from pcl_helper import *

# TODO: Define functions as required

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

    # TODO: Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg)

    # TODO: Voxel Grid Downsampling
    vox = cloud.make_voxel_grid_filter()
    LEAF_SIZE = 0.01
    vox.set_leaf_size(LEAF_SIZE,LEAF_SIZE,LEAF_SIZE)
    cloud_filtered = vox.filter()
    # TODO: PassThrough Filter
    passthrough = cloud_filtered.make_passthrough_filter()
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.6
    axis_max = 1.1
    passthrough.set_filter_limits(axis_min,axis_max)
    
    # TODO: RANSAC Plane Segmentation
    cloud_filtered = passthrough.filter()
    seg = cloud_filtered.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)

    # TODO: Extract inliers and outliers
    max_distance = 0.01
    seg.set_distance_threshold(max_distance)
    inliers, coefficients = seg.segment()
    extracted_inliers = cloud_filtered.extract(inliers,negative=False)
    extracted_outliers = cloud_filtered.extract(inliers,negative=True)
    #outlier removal filter
    #outlier_filter = cloud_filtered.make_statistical_outlier_filter()
    #set the number of neighboring points to analyze for any given point
    #outlier_filter.set_mean_k(50)
    #set threshold scale factor
    #x = 1.0
    #any point with a mean distance larger then 
    #global (mean sitance+x*std_dev) will be considered outlier
    #outlier_filter.set_std_dev_mul_thresh(x)
    #finally call the filter function for magic
    #cloud_filtered = outlier_filter.filter()

    # TODO: Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(extracted_outliers)
    tree = white_cloud.make_kdtree()
    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.010)
    ec.set_MinClusterSize(56.0)
    ec.set_MaxClusterSize(200)
    # search the k-d tree for cluster
    #tree.set_SearchMethod(ec)
    #cluster_indexes = tree.Extract()

    ec.set_SearchMethod(tree)
    cluster_indexes = ec.Extract()
    
    cluster_color = get_color_list(len(cluster_indexes))
    color_cluster_point_list = []
    for j, indexes in enumerate(cluster_indexes):
        for i, index in enumerate(indexes):
            color_cluster_point_list.append([white_cloud[index][0],
                                             white_cloud[index][1],
                                             white_cloud[index][2],
                                             rgb_to_float(cluster_color[j])])
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

                                   
    # TODO: Convert PCL data to ROS messages
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)
    ros_cloud_table = pcl_to_ros(extracted_inliers)
    ros_cloud_objects = pcl_to_ros(extracted_outliers)
    # TODO: Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster.publish(ros_cluster_cloud)
    return

if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('clustering',anonymous=True)
    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/sensor_stick/point_cloud",pc2.PointCloud2,pcl_callback,queue_size=1)
    # TODO: Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects",PointCloud2,queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table",PointCloud2,queue_size=1)
    pcl_cluster = rospy.Publisher("/pcl_cluster",PointCloud2,queue_size=1)
    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()

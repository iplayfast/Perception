#!/usr/bin/env python

import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder

import pickle

from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker

from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Callback function for your Point Cloud Subscriber
def pcl_callback(ros_msg):

# Exercise-2 TODOs:

    # TODO: Convert ROS msg to PCL data
    pcl_msg = ros_to_pcl(ros_msg)
    pcl_raw_pub.publish(pcl_msg);
    # TODO: Voxel Grid Downsampling
    vox = pcl_msg.make_voxel_grid_filter()
    LEAF_SIZE = 0.01
    vox.set_leaf_size(LEAF_SIZE,LEAF_SIZE,LEAF_SIZE)
    cloud_filtered = vox.filter()

    # TODO: PassThrough Filter
    outlier_filter = cloud_filtered.make_statistical_outlier_filter()
    #set the number of neighboring points to analyze for any given point
    outlier_filter.set_mean_k(5)
    #set threshold scale factor
    x = 0.1
    # any point with a mean distance larger then global (mean
    # distance+x*std_dev) will be considered outlier
    outlier_filter.set_std_dev_mul_thresh(x)
    outliers_removed = outlier_filter.filter()

    passthrough = outliers_removed.make_passthrough_filter()
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.6
    axis_max = 1.1
    passthrough.set_filter_limits(axis_min,axis_max)
    passed = passthrough.filter()
    #assign axis and range to the pasthrough filter object
    passthrough = passed.make_passthrough_filter()
    filter_axis = 'y'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = -0.45
    axis_max = 0.45
    passthrough.set_filter_limits(axis_min,axis_max)
    cloud = passthrough.filter()
    # TODO: RANSAC Plane Segmentation
    seg = passed.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)


    # TODO: Extract inliers and outliers
    max_distance = 0.01
    seg.set_distance_threshold(max_distance)
    inliers, coefficients = seg.segment()
    cloud_table = cloud.extract(inliers,negative=False)
    cloud_objects = cloud.extract(inliers,negative=True)
 
    # TODO: Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    tree = white_cloud.make_kdtree()
    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    ec = white_cloud.make_EuclideanClusterExtraction()
    #ec.set_ClusterTolerance(0.010) # or LEAF_SIZE*2
    #ec.set_MinClusterSize(56.0)     # or 10
    #ec.set_MaxClusterSize(200)      # or 2500
    ec.set_ClusterTolerance(LEAF_SIZE*2) # or LEAF_SIZE*2
    ec.set_MinClusterSize(10.0)     # or 10
    ec.set_MaxClusterSize(2500)      # or 2500
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
    ros_outliers_removed = pcl_to_ros(outliers_removed)
    ros_passed = pcl_to_ros(passed)
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cloud_table = pcl_to_ros(cloud_table)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)
 
    # TODO: Publish ROS messages
    pcl_outliers_removed_pub.publish(ros_outliers_removed)
    pcl_passed_pub.publish(ros_passed)
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cluster_cloud)
 
# Exercise-3 TODOs: 

    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects_labels = []
    detected_objects = []
    for index, pts_list in enumerate(cluster_indexes):
        # Grab the points for the cluster
        pcl_cluster= cloud_objects.extract(pts_list)
        #convert pcl to ros
        ros_cluster = pcl_to_ros(pcl_cluster)
        # Extract histogram features
        chists = compute_color_histograms(ros_cluster, using_hsv=True)
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))
        # Compute the associated feature vector
        # Make the prediction
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0] 
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label,label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)
        rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels),
                                          detected_objects_labels))
    # Publish the list of detected objects
        detected_objects_pub.publish(detected_objects)

if __name__ == '__main__':
    # TODO: ROS node initialization
    rospy.init_node('clustering',anonymous=True)
    # TODO: Create Subscribers
    pcl_sub=rospy.Subscriber("/pr2/world/points",pc2.PointCloud2,pcl_callback,queue_size=1)
    # TODO: Create Publishers
    pcl_raw_pub = rospy.Publisher("/pcl_raw",PointCloud2,queue_size=1)
    pcl_outliers_removed_pub = rospy.Publisher("/pcl_outliers_removed",PointCloud2,queue_size=1)
    pcl_passed_pub = rospy.Publisher("/pcl_passed",PointCloud2,queue_size=1)
    object_markers_pub = rospy.Publisher("/object_markers",Marker,queue_size=10)
    detected_objects_pub = rospy.Publisher("/detected_objects",DetectedObjectsArray,queue_size=10)

    pcl_objects_pub = rospy.Publisher("/pcl_objects",PointCloud2,queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table",PointCloud2,queue_size=1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster",PointCloud2,queue_size=1)

    # TODO: Load Model From disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']
    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()


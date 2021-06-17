#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/PointCloud2.h"
#include "rospy_tutorials/Floats.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <chrono>  // for high_resolution_clock

using namespace std;


class mesh2cloud_class {
public:
    mesh2cloud_class() : cloud(new pcl::PointCloud <pcl::PointXYZ>) {
        sub_scene_cloud = n_.subscribe("/camera/depth_registered/points", 1, &mesh2cloud_class::cloud_cb, this);
        sub_box = n_.subscribe("/bbox", 1, &mesh2cloud_class::bbox_cb, this);
        sub_pose = n_.subscribe("/predicted_pose_np", 1, &mesh2cloud_class::pose_cb, this);
        sub_ = n_.subscribe("/predicted_voxel_np", 1, &mesh2cloud_class::chatterCallback, this);
        pub_ = n_.advertise<sensor_msgs::PointCloud2>("predicted_mug_cloud", 1);
        world_frame = "odom_combined";
        camera_frame = "head_mount_asus_rgb_optical_frame";
        isBboxHere = false;
        isCloudHere = false;
        isAziHere = false;
        centroidComputed = false;
        x_mean = 0;
        y_mean = 0;
        z_mean = 0;
        azimuth_rad = 0;
    }

    void pose_cb(const rospy_tutorials::Floats::ConstPtr &msg) {
        cout << "pose: " << msg->data[0] << " " << msg->data[1] << endl;
        azimuth_rad = msg->data[0];
        isAziHere = true;

    }

    void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr &ros_cloud) {
        pcl::fromROSMsg(*ros_cloud, *cloud);
        cout << "cloud conversion done" << endl;
        isCloudHere = true;

    }

    void computeCentroidInScene() {
        if (isBboxHere && isCloudHere) {
            pcl::PointXYZ p_mean;
            int count = 0;
            double x_p = 0;
            double y_p = 0;
            double z_p = 0;
            for (int x = bbox[0]; x <= bbox[2]; x++) {
                for (int y = bbox[1]; y <= bbox[3]; y++) {
                    pcl::PointXYZ p = cloud->at(x, y);
                    if (std::isnan(p.x)) {
                        continue;
                    }
                    x_p += p.x;
                    y_p += p.y;
                    z_p += p.z;
                    count += 1;
                }
            }
            x_mean = x_p / double(count);
            y_mean = y_p / double(count);
            z_mean = z_p / double(count);
            centroidComputed = true;
            cout << "mean: " << x_mean << " " << y_mean << " " << z_mean << "" << endl;
        }
    }

    void bbox_cb(const rospy_tutorials::Floats::ConstPtr &msg) {
        //format [x1, y1, x2, y2] eg [300 131 358 211]
        cout << "bbox: " << msg->data[0] << " " << msg->data[1] << " " << msg->data[2] << " " << msg->data[3] << endl;
        bbox = msg->data;
        isBboxHere = true;
        computeCentroidInScene();
    }

    int get_index(int x, int y, int z) {
        int wxh = 32 * 32;
        int index = x * wxh + z * 32 + y;
        return index;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr binvoxToPCL(std::vector<float> vox) {
        int res_factor = 3;
        //convert voxels to point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        int depth = 32;
        int height = 32;
        int width = 32;
        int scale = 1;
        int tx = 0;
        int tz = 0;
        int ty = 0;
        for (int x = 0; x < depth - 1; x++) {
            for (int y = 0; y < width - 1; y++) {
                for (int z = 0; z < height - 1; z++) {
                    //get indexes of cube
                    vector<int> indexes;
                    for (int i = 0; i <= 1; i++) {
                        for (int j = 0; j <= 1; j++) {
                            for (int k = 0; k <= 1; k++) {
                                indexes.push_back(get_index(x + i, y + j, z + k));
                            }
                        }
                    }
                    //create points evenly distributed inside of cube
                    for (int i = 0; i < res_factor; i++) {
                        for (int j = 0; j < res_factor; j++) {
                            for (int k = 0; k < res_factor; k++) {
                                Eigen::Vector3f pnt;
                                pnt[0] = (float) x + ((float) i) / ((float) res_factor);
                                pnt[1] = (float) y + ((float) j) / ((float) res_factor);
                                pnt[2] = (float) z + ((float) k) / ((float) res_factor);
                                //check if pnt is closer to surface then outside
                                float val = 0.0;
                                for (int n = 0; n < 8; n++) {
                                    //get distance to point (l1 norm)
                                    int a = n / 4;
                                    int b = (n % 4) / 2;
                                    int c = (n % 4) % 2;
                                    float dist = fabs(pnt[0] - (float) (x + a));
                                    dist += fabs(pnt[1] - (float) (y + b));
                                    dist += fabs(pnt[2] - (float) (z + c));
                                    dist += 0.00001;
                                    float weight = (float) (2 * vox[indexes[n]] - 1);
                                    val += (weight / dist);
                                }
                                if (val > 0.0) {
                                    pcl::PointXYZ p;
                                    p.x = (pnt[0] + 0.5) * scale / ((float) depth) + tx;
                                    p.y = (pnt[1] + 0.5) * scale / ((float) width) + ty;
                                    p.z = (pnt[2] + 0.5) * scale / ((float) height) + tz;
                                    cloud->push_back(p);
                                }
                            }
                        }
                    }//finished generate points for cube x,y,z
                }
            }
        }//finished generating point cloud

        return cloud;
    }

    void chatterCallback(const rospy_tutorials::Floats::ConstPtr &msg) {
        auto start_global = std::chrono::high_resolution_clock::now();
        Eigen::Matrix4f transformEigen = Eigen::Matrix4f::Identity();
        tf::StampedTransform transformMsg;
        _tf_listener.waitForTransform(world_frame, camera_frame,
                                      ros::Time::now(), ros::Duration(3.0));
        _tf_listener.lookupTransform(world_frame, camera_frame,
                                     ros::Time(0), transformMsg);
        pcl_ros::transformAsMatrix(transformMsg, transformEigen);
        auto start_1 = std::chrono::high_resolution_clock::now();
        pcl::PointCloud<pcl::PointXYZ>::Ptr predictCloud = binvoxToPCL(msg->data);
        auto finish_1 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed_1 = finish_1 - start_1;
        std::cout << "Elapsed time for binvoxToPCL: " << elapsed_1.count() << " s\n";
        auto start_2 = std::chrono::high_resolution_clock::now();
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        float my_scale = 0.13;
        transform(0, 0) = transform(0, 0) * my_scale;
        transform(1, 1) = transform(1, 1) * my_scale;
        transform(2, 2) = transform(2, 2) * my_scale;
        Eigen::Affine3f transform_affine(transform);
        Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
        //rotate back to world coordinate (we rotated the models for our network)
        float theta = M_PI / 2;
        transform_2.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitY()));
        theta = M_PI;
        Eigen::Affine3f transform_3 = Eigen::Affine3f::Identity();
        transform_3.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitX()));
        theta = M_PI / 2;
        Eigen::Affine3f transform_5 = Eigen::Affine3f::Identity();
        transform_5.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));
        //now we want to center the object, compute translation
        pcl::PointXYZ minPt, maxPt;
        pcl::getMinMax3D(*predictCloud, minPt, maxPt);
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*predictCloud, centroid);
        Eigen::Affine3f transform_4 = Eigen::Affine3f::Identity();
        transform_4.translation() << -centroid[0], -centroid[1], -minPt.z;
        Eigen::Affine3f my_transforms = transform_4 * transform_5 * transform_3 * transform_2 * transform_affine;
        pcl::transformPointCloud(*predictCloud, *predictCloud, my_transforms);

        if (isAziHere) {
            Eigen::Affine3f transform_azi = Eigen::Affine3f::Identity();
            transform_azi.rotate(Eigen::AngleAxisf(-azimuth_rad, Eigen::Vector3f::UnitZ()));
            pcl::transformPointCloud(*predictCloud, *predictCloud, transform_azi);
        }
        //now we want to transform it to the table, compute difference between centroids
        if (centroidComputed) {
            pcl::compute3DCentroid(*predictCloud, centroid);
            Eigen::Affine3f transform_6 = Eigen::Affine3f::Identity();
            Eigen::Vector4f pcl_centroid(x_mean, y_mean, z_mean, 1);
            Eigen::Vector4f transformed_centroid = transformEigen * pcl_centroid;
            transform_6.translation() << transformed_centroid[0] - centroid[0] - 0.01, transformed_centroid[1] -
                                                                                       centroid[1] + 0.01,
                    transformed_centroid[2] - centroid[2] + 0.01;
            pcl::transformPointCloud(*predictCloud, *predictCloud, transform_6);
        }
        auto finish_2 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed_2 = finish_2 - start_2;
        std::cout << "Elapsed time for pcl transformations: " << elapsed_2.count() << " s\n";
        sensor_msgs::PointCloud2 ros_cloud;
        pcl::toROSMsg(*predictCloud, ros_cloud);
        ros_cloud.header.stamp = ros::Time::now();
        ros_cloud.header.frame_id = world_frame;
        pub_.publish(ros_cloud);
        auto finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = finish - start_global;
        std::cout << "Elapsed time for whole callback: " << elapsed.count() << " s\n";
    }

private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    ros::Subscriber sub_box;
    ros::Subscriber sub_pose;
    ros::Subscriber sub_scene_cloud;
    tf::TransformListener _tf_listener;
    std::string world_frame;
    std::string camera_frame;
    vector<float> bbox;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    bool isCloudHere;
    bool isBboxHere;
    bool isAziHere;
    double x_mean;
    double y_mean;
    double z_mean;
    bool centroidComputed;
    float azimuth_rad;

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "mesh2_cloud_node");
    mesh2cloud_class mesh2cloud;
    ros::spin();
    return 0;
}

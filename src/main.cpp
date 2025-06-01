#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/filters/extract_indices.h> // Added for pcl::ExtractIndices
#include <pcl/visualization/pcl_visualizer.h> // Commented out for headless testing
#include <pcl/console/parse.h>

// Placeholder for configuration file loading
void loadConfig(const std::string& config_path) {
    // In the future, this function will load parameters from a config file
    std::cout << "Config file loading is not yet implemented. Using default parameters." << std::endl;
}

int main(int argc, char** argv) {
    // --- 1. Parse command line arguments ---
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <input_point_cloud_file> [config_file_path]" << std::endl;
        return -1;
    }
    std::string file_path = argv[1];
    std::string config_path = "";
    if (argc == 3) {
        config_path = argv[2];
    }

    // --- 2. Load configuration (placeholder) ---
    if (!config_path.empty()) {
        loadConfig(config_path);
    } else {
        std::cout << "No config file provided. Using default parameters." << std::endl;
    }

    // --- 3. Load point cloud ---
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::string extension = file_path.substr(file_path.find_last_of(".") + 1);

    if (extension == "pcd") {
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, *cloud) == -1) {
            PCL_ERROR("Couldn't read file %s\n", file_path.c_str());
            return (-1);
        }
    } else if (extension == "ply") {
        if (pcl::io::loadPLYFile<pcl::PointXYZ>(file_path, *cloud) == -1) {
            PCL_ERROR("Couldn't read file %s\n", file_path.c_str());
            return (-1);
        }
    } else {
        PCL_ERROR("Unsupported file format: %s. Please use .pcd or .ply\n", extension.c_str());
        return (-1);
    }
    std::cout << "Loaded " << cloud->width * cloud->height << " data points from " << file_path << std::endl;

    // --- 4. Display original point cloud (Commented out for headless testing) ---
    pcl::visualization::PCLVisualizer::Ptr viewer_original(new pcl::visualization::PCLVisualizer("Original Cloud"));
    viewer_original->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> original_color(cloud, 255, 255, 255); // White
    viewer_original->addPointCloud<pcl::PointXYZ>(cloud, original_color, "original cloud");
    viewer_original->addCoordinateSystem(1.0);
    viewer_original->initCameraParameters();

    std::cout << "Visualizing original cloud. Close the viewer to continue." << std::endl;
    while (!viewer_original->wasStopped()) {
        viewer_original->spinOnce(100);
    }
    viewer_original->close();

    // --- 5. Apply Progressive Morphological Filter ---
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointIndicesPtr ground_indices(new pcl::PointIndices);

    pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
    pmf.setInputCloud(cloud);
    pmf.setMaxWindowSize(20);        // Set reasonable default
    pmf.setSlope(1.0f);              // Set reasonable default
    pmf.setInitialDistance(0.5f);    // Set reasonable default
    pmf.setMaxDistance(3.0f);        // Set reasonable default
    pmf.extract(ground_indices->indices);

    // Create a point cloud containing only non-ground points
    pcl::PointCloud<pcl::PointXYZ>::Ptr non_ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(ground_indices);
    extract.setNegative(true); // True means extract points NOT in indices
    extract.filter(*non_ground_cloud);

    std::cout << "Ground removal complete. Original points: " << cloud->size() << ", Ground points: " << ground_indices->indices.size() << ", Non-ground points: " << non_ground_cloud->size() << std::endl;

    // --- 6. Display filtered point cloud (Commented out for headless testing) ---
    pcl::visualization::PCLVisualizer::Ptr viewer_filtered(new pcl::visualization::PCLVisualizer("Filtered Cloud (Non-Ground)"));
    viewer_filtered->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> filtered_color(non_ground_cloud, 0, 255, 0); // Green
    viewer_filtered->addPointCloud<pcl::PointXYZ>(non_ground_cloud, filtered_color, "filtered cloud");
    viewer_filtered->addCoordinateSystem(1.0);
    viewer_filtered->initCameraParameters();

    std::cout << "Visualizing filtered cloud. Close the viewer to continue." << std::endl;
    while (!viewer_filtered->wasStopped()) {
        viewer_filtered->spinOnce(100);
    }
    viewer_filtered->close();

    // --- 7. Save filtered point cloud ---
    if (non_ground_cloud->points.empty()) {
        std::cerr << "Filtered cloud is empty, not saving." << std::endl;
    } else {
        pcl::io::savePCDFileASCII("filtered_cloud.pcd", *non_ground_cloud);
        std::cout << "Saved " << non_ground_cloud->points.size() << " data points to filtered_cloud.pcd." << std::endl;
    }

    return 0;
}

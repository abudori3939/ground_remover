#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/filters/extract_indices.h> // Added for pcl::ExtractIndices
#include <pcl/filters/voxel_grid.h> // Added for VoxelGrid downsampling
#include <pcl/visualization/pcl_visualizer.h> // Commented out for headless testing
#include <pcl/console/parse.h>
#include <fstream> // Required for std::ifstream
#include <string>  // Required for std::string, std::getline, std::stof
#include <limits>  // Required for std::numeric_limits

// 設定ファイルを読み込む関数
// config_path: 設定ファイルのパス
// key: 読み込む設定項目のキー
// default_value: キーが見つからない場合や値が無効な場合に返すデフォルト値
// 戻り値: 読み込んだ設定値、またはデフォルト値
float loadConfig(const std::string& config_path, const std::string& key, float default_value) {
    std::ifstream config_file(config_path);
    std::string line;

    // ファイルが開けるか確認
    if (!config_file.is_open()) {
        std::cout << "設定ファイルを開けませんでした: " << config_path << "。デフォルト値 (" << default_value << ") を使用します。" << std::endl;
        return default_value;
    }

    // ファイルを一行ずつ読み込む
    while (std::getline(config_file, line)) {
        // コメント行と空行をスキップ
        if (line.empty() || line[0] == '#') {
            continue;
        }

        // キーを探す (キーの形式は "key: value")
        size_t delimiter_pos = line.find(':');
        if (delimiter_pos != std::string::npos) {
            std::string current_key = line.substr(0, delimiter_pos);
            // 前後の空白をトリム (簡易的なトリム)
            current_key.erase(0, current_key.find_first_not_of(" \t"));
            current_key.erase(current_key.find_last_not_of(" \t") + 1);

            if (current_key == key) {
                std::string value_str = line.substr(delimiter_pos + 1);
                // 値の文字列から前後の空白をトリム (簡易的なトリム)
                value_str.erase(0, value_str.find_first_not_of(" \t"));
                value_str.erase(value_str.find_last_not_of(" \t") + 1);

                try {
                    float found_value = std::stof(value_str);
                    std::cout << "設定ファイルから読み込みました: " << key << " = " << found_value << std::endl;
                    config_file.close();
                    return found_value;
                } catch (const std::invalid_argument& ia) {
                    std::cout << "キー '" << key << "' の値が無効です: " << value_str << "。デフォルト値 (" << default_value << ") を使用します。" << std::endl;
                    config_file.close();
                    return default_value;
                } catch (const std::out_of_range& oor) {
                    std::cout << "キー '" << key << "' の値が範囲外です: " << value_str << "。デフォルト値 (" << default_value << ") を使用します。" << std::endl;
                    config_file.close();
                    return default_value;
                }
            }
        }
    }

    // キーが見つからなかった場合
    std::cout << "キー '" << key << "' が設定ファイルに見つかりませんでした。デフォルト値 (" << default_value << ") を使用します。" << std::endl;
    config_file.close();
    return default_value;
}

/* Placeholder for configuration file loading
void loadConfig(const std::string& config_path) {
    // In the future, this function will load parameters from a config file
    std::cout << "Config file loading is not yet implemented. Using default parameters." << std::endl;
}
*/

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

    // --- 2. Load configuration ---
    float downsample_leaf_size = 0.1f; // Default value
    if (!config_path.empty()) {
        downsample_leaf_size = loadConfig(config_path, "downsample_leaf_size", downsample_leaf_size);
        // ここで他の設定も同様に読み込むことができます
        //例: max_window_size = loadConfig(config_path, "max_window_size", 20.0f);
    } else {
        std::cout << "設定ファイルが提供されていません。デフォルトの 'downsample_leaf_size' (" << downsample_leaf_size << ") を使用します。" << std::endl;
    }
    // 読み込んだ値を使用する例 (現在は未使用)
    std::cout << "Downsample leaf size: " << downsample_leaf_size << std::endl;

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

    // --- 3.5 ダウンサンプリングの実行 ---
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (downsample_leaf_size > 0.0f) { // 0以下の場合はスキップ
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(cloud);
        voxel_filter.setLeafSize(downsample_leaf_size, downsample_leaf_size, downsample_leaf_size);
        voxel_filter.filter(*downsampled_cloud);

        std::cout << "ダウンサンプリング前ポイント数: " << cloud->size() << std::endl;
        std::cout << "ダウンサンプリング後ポイント数: " << downsampled_cloud->size() << std::endl;
    } else {
        std::cout << "リーフサイズが無効なため、ダウンサンプリングをスキップしました。オリジナルを使用します。" << std::endl;
        downsampled_cloud = cloud; // ダウンサンプリングしない場合はオリジナルクラウドを使用
    }


    // --- 4. Display original point cloud (Commented out for headless testing, now displays downsampled cloud if active) ---
    // pcl::visualization::PCLVisualizer::Ptr viewer_original(new pcl::visualization::PCLVisualizer("Original (Downsampled) Cloud"));
    // viewer_original->setBackgroundColor(0, 0, 0);
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> downsampled_color_handler(downsampled_cloud, 255, 255, 255); // White
    // viewer_original->addPointCloud<pcl::PointXYZ>(downsampled_cloud, downsampled_color_handler, "original_downsampled_cloud");
    // viewer_original->addCoordinateSystem(1.0);
    // viewer_original->initCameraParameters();

    // std::cout << "Visualizing original cloud. Close the viewer to continue." << std::endl;
    // while (!viewer_original->wasStopped()) {
    //     viewer_original->spinOnce(100);
    // }
    // viewer_original->close();

    // --- 5. Apply Progressive Morphological Filter ---
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointIndicesPtr ground_indices(new pcl::PointIndices);

    pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
    // ダウンサンプリングされたクラウドをフィルタの入力として設定
    pmf.setInputCloud(downsampled_cloud);
    pmf.setMaxWindowSize(20);        // Set reasonable default
    pmf.setSlope(1.0f);              // Set reasonable default
    pmf.setInitialDistance(0.5f);    // Set reasonable default
    pmf.setMaxDistance(3.0f);        // Set reasonable default
    pmf.extract(ground_indices->indices);

    // Create a point cloud containing only non-ground points
    pcl::PointCloud<pcl::PointXYZ>::Ptr non_ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    // ダウンサンプリングされたクラウドを抽出処理の入力として設定
    extract.setInputCloud(downsampled_cloud);
    extract.setIndices(ground_indices);
    extract.setNegative(true); // True means extract points NOT in indices
    extract.filter(*non_ground_cloud);

    std::cout << "Ground removal complete. Points after downsampling: " << downsampled_cloud->size() << ", Ground points: " << ground_indices->indices.size() << ", Non-ground points: " << non_ground_cloud->size() << std::endl;

    // --- 6. Display filtered point cloud (Commented out for headless testing) ---
    // pcl::visualization::PCLVisualizer::Ptr viewer_filtered(new pcl::visualization::PCLVisualizer("Filtered Cloud (Non-Ground)"));
    // viewer_filtered->setBackgroundColor(0, 0, 0);
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> filtered_color(non_ground_cloud, 0, 255, 0); // Green
    // viewer_filtered->addPointCloud<pcl::PointXYZ>(non_ground_cloud, filtered_color, "filtered cloud");
    // viewer_filtered->addCoordinateSystem(1.0);
    // viewer_filtered->initCameraParameters();

    // std::cout << "Visualizing filtered cloud. Close the viewer to continue." << std::endl;
    // while (!viewer_filtered->wasStopped()) {
    //     viewer_filtered->spinOnce(100);
    // }
    // viewer_filtered->close();

    // --- 7. Save filtered point cloud ---
    if (non_ground_cloud->points.empty()) {
        std::cerr << "Filtered cloud is empty, not saving." << std::endl;
    } else {
        pcl::io::savePCDFileASCII("filtered_cloud.pcd", *non_ground_cloud);
        std::cout << "Saved " << non_ground_cloud->points.size() << " data points to filtered_cloud.pcd." << std::endl;
    }

    return 0;
}

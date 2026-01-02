#include <Zivid/Experimental/LocalPointCloudRegistrationParameters.h>
#include <Zivid/Experimental/Toolbox/PointCloudRegistration.h>
#include <Zivid/Experimental/PointCloudExport.h>
#include <Zivid/Visualization/Visualizer.h>
#include <Zivid/Zivid.h>

#include <Eigen/Core>

#include <cmath>
#include <filesystem>
#include <iostream>
#include <vector>

#define ZIVID_SAMPLE_DATA_DIR "C:\\Users\\ESCL\\Documents\\ZividStuff\\owl_scans"

namespace
{
    void visualizePointCloud(const Zivid::UnorganizedPointCloud &unorganizedPointCloud)
    {
        Zivid::Visualization::Visualizer visualizer;

        visualizer.showMaximized();
        visualizer.show(unorganizedPointCloud);
        visualizer.resetToFit();

        std::cout << "Running visualizer. Blocking until window closes." << std::endl;
        visualizer.run();
    }
} // namespace

int main () {
    Zivid::Application app;

    std::cout << "running" << std::endl;

    const auto directory = std::filesystem::path(ZIVID_SAMPLE_DATA_DIR);

    if(!std::filesystem::exists(directory))
    {
        std::ostringstream oss;
        oss << "Missing dataset folders.\n"
            << "Make sure 'StitchingPointClouds/BlueObject/' exist at " << ZIVID_SAMPLE_DATA_DIR << ".\n\n"
            << "You can download the dataset (StitchingPointClouds.zip) from:\n"
            << "https://support.zivid.com/en/latest/api-reference/samples/sample-data.html";

        throw std::runtime_error(oss.str());
    }

    int number_frames = 21;

    std::vector<Zivid::Frame> frames(number_frames);

    for (int i = 0; i < number_frames; i++) {
        frames[i] = Zivid::Frame((directory / ("owl" + std::to_string(i+1) + ".zdf")).string());
    }
    
    Zivid::UnorganizedPointCloud unorganizedNotStitchedPointCloud;

    std::vector<Zivid::UnorganizedPointCloud> unorganizedPointClouds(number_frames);
    for (int i = 0; i < number_frames; i++) {
        unorganizedPointClouds[i] = frames[i].pointCloud().toUnorganizedPointCloud().voxelDownsampled(1.0, 3);
        unorganizedNotStitchedPointCloud.extend(unorganizedPointClouds[i]);
    }

    std::cout << "Displaying point clouds before stitching" << std::endl;
    visualizePointCloud(unorganizedNotStitchedPointCloud);

    std::cout << "Estimating transformation between point clouds" << std::endl;
    
    Zivid::UnorganizedPointCloud unorganizedStitchedPointCloud;
    auto registrationParams = Zivid::Experimental::LocalPointCloudRegistrationParameters{};
    auto previousToCurrentPointCloudTransform = Zivid::Matrix4x4::identity();

    for(int i = 0; i < number_frames; i++)
    {
        const auto unorganizedPointCloud = frames[i].pointCloud().toUnorganizedPointCloud().voxelDownsampled(1.0, 2);

        if(i != 0)
        {
            if(unorganizedStitchedPointCloud.size() < 4 || unorganizedPointCloud.size() < 4)
            {
                std::cout << "Not enough points for registration, skipping stitching..." << std::endl;
                continue;
            }

            const auto registrationResult = Zivid::Experimental::Toolbox::localPointCloudRegistration(
                unorganizedStitchedPointCloud,
                unorganizedPointCloud,
                registrationParams,
                previousToCurrentPointCloudTransform);

            if(!registrationResult.converged())
            {
                std::cout << "Registration did not converge..." << std::endl;
                continue;
            }

            previousToCurrentPointCloudTransform = registrationResult.transform().toMatrix();
            unorganizedStitchedPointCloud.transform(previousToCurrentPointCloudTransform.inverse());
        }

        unorganizedStitchedPointCloud.extend(unorganizedPointCloud);
        std::cout << "Captures done: " << i << std::endl;
    }

    // save point cloud

    const auto outputFilePath = directory / "owl_stitched_point_cloud.zdf";
    std::cout << "Saving stitched point cloud to " << outputFilePath << std::endl;

    visualizePointCloud(unorganizedStitchedPointCloud);
    Zivid::Experimental::PointCloudExport::exportUnorganizedPointCloud(unorganizedStitchedPointCloud, Zivid::Experimental::PointCloudExport::FileFormat::PLY{ "owl_composite.ply" });
}

#include<iostream>
#include<opencv2/opencv.hpp>

#include "../include/myppf/headers.hpp"

using namespace std;
using namespace myppf;

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointNormal>::Ptr PCNormals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_model(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_scene(new pcl::PointCloud<pcl::PointXYZRGB>);

    string empty_scene = "../data/empty.ply";
    //string full_scene = "../data/scene_remove.ply";
    string model_path= "../data/box.ply";
    //string model_path = "../data/box.stl";
    //string model_path = argv[1];
    //string full_scene = argv[2];
    //string empty_scene = argv[3];

    /****input parameter file****/
    string strSettingPath = "../para/settings.yaml";

    /****load point cloud****/
    pcl::io::loadPLYFile(model_path.c_str(), *PCNormals);
    int model_num = PCNormals->points.size();

    /****create PPFdetector to trainmodel****/
    PPFDetector detector(strSettingPath);
    detector.trainModel(model_path);
    //detector.save_model("model.txt");
    //detector.load_model("model.txt");


    /*compute batch PPF*/ 

    const int datanum = std::stoi(argv[1]);//set data number
    double ADDsum;//auc initial
    int count = 0;
    double time = 0;//compute time
    std::string front_name = "../../data/pointcloud/";
    for(int i = 1; i <= datanum; i++)
    {
        auto number = std::to_string(i);
        std::string scene_name = front_name + number + ".ply";
        std::string output_name = "../data/output/" + number + ".txt";
        
        pcl::io::loadPLYFile(scene_name.c_str(), *colored_scene);

        detector.setScene(empty_scene, scene_name, model_path);
        auto starttime = std::chrono::system_clock::now();
        detector.match();
        detector.rescore();
        detector.rescoreAfterICP();
        std::chrono::duration<double> difftime = std::chrono::system_clock::now() - starttime;
        std::cout << "cost time: " << difftime.count() << "s" << std::endl;
        time += difftime.count();

        //get pose
        Pose3DPtr p = detector.getPoseAfterICP();
        Eigen::Matrix4d output_pose = p->pose.matrix();
        
        double addscore = detector.getScore(p, model_path.c_str(), scene_name.c_str(), 0.001);
        std::cout << "add score: " << addscore << std::endl;
        double grade = (addscore*100)/model_num;

        if(grade > 80)
        {
            ADDsum += grade;
            count++;
        }
        std::cout << "add per: " << grade << std::endl;
        
        std::ofstream real;
        real.open(output_name, std::ios::out|std::ios::app);
        real << output_pose << endl;
        real << "add per:" << grade << endl;
        real.close();
    }

    std::string dataname = "../data/output/perdata.txt";
    std::ofstream Fp;
    Fp.open(dataname, std::ios::out|std::ios::app);
    auto pertime = time/datanum;
    std::cout << "cost per time: " << pertime << "s" << std::endl;

    double addper = (ADDsum/count);
    std::cout << "ADD PERCENT:" << addper << std::endl;
    Fp << "pertime: " << pertime << endl;
    Fp << "peradd: " << addper << endl;
    Fp.close();
   
    /*
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud(*colored_model,*transformed,Tfinal);
    for(int i=0; i<transformed->points.size(); i++)
    {
        colored_scene->points.push_back(transformed->points[i]);
    }
    //point cloud viewer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));
    viewer->addPointCloud<pcl::PointXYZRGB>(colored_scene,"matching");
    
    while(!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::microseconds(100000));
    }
    */
    return 0;
}

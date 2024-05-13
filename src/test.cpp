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

    //string empty_scene = "../data/e1.ply";
    //string full_scene = "../data/box.ply";
    //string model_path= "../data/luigicur.ply";
    //string model_path = "../data/box.stl";
    //string model_path = argv[1];
    //string full_scene = argv[2];
    //string empty_scene = argv[3];

    string datapath = "../para/data.yaml";//file path 
    cv::FileStorage data_path(datapath.c_str(), cv::FileStorage::READ);
    if(!data_path.isOpened())
    {
        cerr << "Failed to open settings file at: " << datapath << endl;
        exit(-1);
    }
    string empty_scene = data_path["emptyscene"];
    string model_path= data_path["modelpath"];



    string strSettingPath = "../para/settings.yaml";//some parameter

    //for visualization
    /*
    loadPLYRGB(colored_scene,full_scene.c_str());
    loadSTL(PCNormals, model_path.c_str(), 1500, 15, 15, 20);
    pcl::io::savePLYFile("./STL.ply", *PCNormals);
    colored_model->points.resize(PCNormals->points.size());
    for(int i=0; i<PCNormals->points.size(); i++)
    {
        colored_model->points[i].x = PCNormals->points[i].x;
        colored_model->points[i].y = PCNormals->points[i].y;
        colored_model->points[i].z = PCNormals->points[i].z;
        colored_model->points[i].r = 255;
        colored_model->points[i].g = 255;
        colored_model->points[i].b = 0;
    }
    */
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
    //std::string input_front = "../../data/pointcloud/";
    string input_front = data_path["inputfront"];
    string output_front = data_path["outputfront"];
    double score_thre = data_path["scorethre"];
    for(int i = 1; i <= datanum; i++)
    {
        auto number = std::to_string(i);
        //std::string scene_name = "../data/44.ply";
        std::string scene_name = input_front + number + ".ply";
        std::string output_name = output_front + number + ".txt";
        
        pcl::io::loadPLYFile(scene_name.c_str(), *colored_scene);
        std::cout << "test" << std::endl;

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
        
        double addscore = detector.getScore(p, model_path.c_str(), scene_name.c_str(), score_thre);
        std::cout << "add score: " << addscore << std::endl;
        double grade = (addscore*100)/model_num;

        if(grade > 10)
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

    return 0;
}

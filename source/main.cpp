#include "../../../Qing/qing_dir.h"
#include "../../../Qing/qing_string.h"

#include "processer.h"
#include "converter.h"

void convert(const string& src_file_name, const string& dst_file_name) {
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr src_pointcloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr dst_pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    pcl::PLYReader reader;
    reader.read(src_file_name, *src_pointcloud);
    int size = src_pointcloud->points.size();
    if(0==size) {
        cerr << "failed to load " << src_file_name << endl;
        return ;
    }

    dst_pointcloud->points.reserve(2*size);
    for(int i = 0; i < size; ++i) {
        pcl::PointXYZRGBNormal src_pt = (*src_pointcloud)[i];
        pcl::PointXYZRGB dst_pt ;
        dst_pt.x = src_pt.x;
        dst_pt.y = src_pt.y;
        dst_pt.z = src_pt.z;
        dst_pt.rgb = src_pt.rgb;
        dst_pointcloud->points.push_back(dst_pt);
    }
    cout << dst_pointcloud->points.size() << " points.." << endl;

    pcl::PLYWriter writer;
    writer.write(dst_file_name, *dst_pointcloud);
    cout << "saving " << dst_file_name << endl;
}

int main(int argc, char * argv[])
{
    cout << "Usage: " << argv[0] << " FRM_0245" << endl;
    if(2!=argc) {
        cerr << "invalid arguments." << endl;
        return -1;
    }

    string frame = argv[1];   //"FRM_0259";
    //string data_folder = "/media/ranqing/QING-WD/ZJU/HumanResults/20161224/scanner_pointclouds/";
    string data_folder = "/media/ranqing/ranqing_wd/ZJU/HumanResults/20161224/scanner_pointclouds/";
    string frame_data_folder = data_folder + frame + "/";
    cout << frame_data_folder << endl;

    string out_data_folder = "./" + frame + "/";
//    string out_data_folder = "/media/ranqing/QING-WD/ZJU/HumanResults/20161224/postprocess_pointclouds/" + frame + "/";
    qing_create_dir(out_data_folder);
    cout << out_data_folder << endl;

    vector<string> all_points_files(0);
    qing_get_all_files(frame_data_folder, all_points_files);
    cout << all_points_files.size() << " files " << endl;

#if 0   //first time will be 1
    for (int i = 0, size = all_points_files.size(); i < size; ++i) {
        string src_file_name = frame_data_folder + all_points_files[i];
        string dst_file_name = out_data_folder + all_points_files[i];
        convert(src_file_name, dst_file_name);
    }
#endif

    PointcloudProcesser * processer = new PointcloudProcesser();
    if(NULL==processer) {
        cerr << "failed to start a point process pipeline...." << endl;
        return -1;
    }

    string ply_file_name, out_file_name;

    for(int i = 0, size = all_points_files.size(); i < size; ++i) {
        ply_file_name = frame_data_folder + all_points_files[i];
        processer->load_ply(ply_file_name);

        if(true == processer->get_is_loaded()) {
          //  processer->down_sampling(2);

            processer->outliers_removal(3);                    //outliers_removal
# if 0
            out_file_name = out_data_folder +  "clean_" + all_points_files[i];
            processer->save_ply(out_file_name);
# endif

            processer->down_sampling(10);           //down_sampling
# if 0
            out_file_name = out_data_folder + "down_" + all_points_files[i];
            processer->save_ply(out_file_name);
# endif

            float radius = 20;
            if(-1 != ply_file_name.find("A01A02")) radius = 25;

            out_file_name = out_data_folder + /*"mls_" + qing_int_2_string((int)radius) +*/ all_points_files[i];
            processer->mls_resampling(radius);
            processer->correct_normal(0.f, 0.f, 0.f);
            processer->save_ply(out_file_name);
        }
        else
        {
            cerr << "zero points...skip point processer..." << endl;
            continue;
        }
    }

    return 1;
}


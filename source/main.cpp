#include "../../../Qing/qing_dir.h"

#include "processer.h"

int main(int argc, char * argv[])
{
    cout << "Usage: " << argv[0] << " FRM_0259" << endl;
    if(2!=argc) {
        cerr << "invalid arguments." << endl;
        return -1;
    }

    string frame = argv[1];   //"FRM_0259";
    string data_folder = "../../HumanDatas_20161224/point_results/";
    string frame_data_folder = data_folder + frame + "/";
    cout << frame_data_folder << endl;

    string out_data_folder = "./" + frame + "/";
    qing_create_dir(out_data_folder);

    vector<string> all_points_files(0);
    qing_get_all_files(frame_data_folder, all_points_files);
    cout << all_points_files.size() << " files " << endl;

    PointcloudProcesser * processer = new PointcloudProcesser();
    if(NULL==processer) {
        cerr << "failed to start a point process pipeline...." << endl;
        return -1;
    }

    for(int i = 0, size = all_points_files.size(); i < size; ++i) {
        string ply_file_name = frame_data_folder + all_points_files[i];
        processer->load_ply(ply_file_name);

        if( true == processer->get_is_loaded() ) {
            string out_file_name = out_data_folder + "sub_" + all_points_files[i];
            processer->down_sampling(out_file_name);
        }
        else {
            cerr << "zero points...skip downsampling..." << endl;
        }

    }

 //   step 1
 //   savename = folder + "downsample_" + filename ;
 //   refiner->downsample(savename);   //downsample

    //step 2 100 1.0, iteration time = 3
//    savename = folder + "clean_" + filename;
//    refiner->removeoutlier(savename);


    //savename = folder + "mls_" + filename;
    //refiner->mlsresample(savename);*/

    return 1;
}


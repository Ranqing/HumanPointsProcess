#include "converter.h"

PointcloudConverter::PointcloudConverter() {
}

PointcloudConverter::~PointcloudConverter() {

}

void PointcloudConverter::load_src_ply(const string infile) {
    cout << "loading " << infile << endl;
    pcl::PLYReader reader;
    reader.read(infile, *m_src_cloud);

    int size = m_src_cloud->width * m_src_cloud->height;
    if(0==size) { m_is_loaded = false; }
    else {m_is_loaded = true;}

    cout << "Pointcloud size: " << m_src_cloud->width * m_src_cloud->height
         << " data points (" << pcl::getFieldsList(*m_cloud) << "). " << endl;
}

void PointcloudConverter::save_dst_ply(const string outfile) {
    pcl::PLYWriter writer;
    writer.write(outfile, *m_dst_cloud);
    cout << "saving " << outfile << endl;
}

void PointcloudConverter::convert() {

    int size = m_src_cloud->points.size();
    m_dst_cloud->points.reserve(2*size);

//    for(int i = 0; i < spoints->points.size(); ++i)
//    {
//        KittiPoint spt = (*spoints)[i];
//        int x = (int)pixels[i](0);
//        int y = (int)pixels[i](1);

//        //coloring
//        cv::Vec3b color = m_image.at<Vec3b>((int)y, (int)x);
//        u_int8_t r = color.val[2];
//        u_int8_t g = color.val[1];
//        u_int8_t b = color.val[0];
//        u_int32_t rgb = ((u_int32_t)r << 16 | (u_int32_t)g << 8 | (u_int32_t)b);

//        spt.rgb = *reinterpret_cast<float*>(&rgb);
//    }
//    KittiPoint kpt;
//    kpt.x = -pt(0);
//    kpt.y = -pt(1);
//    kpt.z = -pt(2);

//    points->push_back(kpt);
}

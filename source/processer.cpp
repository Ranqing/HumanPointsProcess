#include "processer.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>

PointcloudProcesser::PointcloudProcesser(): m_is_loaded(false), m_cloud(new QingPointcloud()), m_cloud_filtered(new QingPointcloud())
{
}

PointcloudProcesser::~PointcloudProcesser() {

}

void PointcloudProcesser::load_ply(const string filename) {
//    m_cloud = new QingPointcloud();
//    m_cloud_filtered = new QingPointcloud();

    m_cloud->clear();
    m_cloud_filtered->clear();

    cout << "loading " << filename << endl;
    pcl::PLYReader reader;
    reader.read(filename, *m_cloud);

    int size = m_cloud->width * m_cloud->height;
    if(0==size) { m_is_loaded = false; }
    else {m_is_loaded = true;}

    cout << "Pointcloud size: " << m_cloud->width * m_cloud->height
         << " data points (" << pcl::getFieldsList(*m_cloud) << "). " << endl;
}

void PointcloudProcesser::save_ply(const string outfile) {
    pcl::PLYWriter writer;
    writer.write(outfile, *m_cloud_filtered);
    cout << "saving " << outfile << endl;
}

void PointcloudProcesser::outliers_removal(int times/* = 1*/) {
    int numk = 60;
    double stddev = 1.0;

    m_cloud_filtered = m_cloud->makeShared();
    while(times--) {
        pcl::StatisticalOutlierRemoval<QingPoint> sor;
        cout << "before outliers removal: " << (m_cloud_filtered->points.size())  << " points." << std::endl;
        sor.setInputCloud(m_cloud);
        sor.setMeanK(numk);
        sor.setStddevMulThresh (stddev);
        sor.filter(*m_cloud_filtered);

        cout << "after: " << (m_cloud_filtered->points.size()) << " points." << std::endl;
        m_cloud = m_cloud_filtered->makeShared();
        numk += 20;
    }
}

void PointcloudProcesser::down_sampling(int scale /*=4*/) {
    m_cloud_filtered = m_cloud->makeShared();
    int src_size = m_cloud->points.size();
    int dst_size = m_cloud_filtered->points.size();

    float leafsize = 0.5f;
    float stepsize = 0.05f;

    pcl::VoxelGrid<QingPoint> sor;
    sor.setInputCloud(m_cloud);
    while((dst_size * scale) > src_size) {
        sor.setLeafSize(leafsize, leafsize, leafsize);
        sor.filter(*m_cloud_filtered);
        dst_size = m_cloud_filtered->points.size();
        cout << "after down-sampling, pointcloud size: " << dst_size << ", " << dst_size * scale <<  "\tsrc_size = " << src_size <<  "\tleafsize = " << leafsize << endl;
        leafsize += stepsize;
    }

    m_cloud = m_cloud_filtered->makeShared();
}

void PointcloudProcesser::mls_resampling(float radius) {
    pcl::search::KdTree<QingPoint>::Ptr tree(new pcl::search::KdTree<QingPoint>());
    pcl::MovingLeastSquares<QingPoint, QingPoint> mls;

    mls.setComputeNormals(true);
    mls.setInputCloud(m_cloud);
    mls.setPolynomialFit(true);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(radius);
    mls.process(*m_cloud_filtered);

    cout << "after msl_resample: " << m_cloud_filtered->points.size() << endl;
    m_cloud = m_cloud_filtered->makeShared();

}

void PointcloudProcesser::correct_normal(float vx, float vy, float vz) {
    m_cloud_filtered = m_cloud->makeShared();

    int size = m_cloud_filtered->points.size();
    float v_px, v_py, v_pz, dot_val;
    float nx, ny, nz;

    for(int i = 0; i < size; ++i) {
        QingPoint pt = m_cloud_filtered->points[i];

        v_px = vx - pt.x ;
        v_py = vy - pt.y ;
        v_pz = vz - pt.z ;
        nx = pt.normal[0];
        ny = pt.normal[1];
        nz = pt.normal[2];

        dot_val = v_px * nx + v_py * ny + v_pz * nz;
        if(dot_val < 0) {
            m_cloud_filtered->points[i].normal[0] = -nx;
            m_cloud_filtered->points[i].normal[1] = -ny;
            m_cloud_filtered->points[i].normal[2] = -nz;
        }
    }
    m_cloud = m_cloud_filtered->makeShared();
}

//
// Created by wuwei on 17-8-8.
//
#include "UAVDenseProcess.h"
#include "UAVCommon.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"
#include <boost/program_options.hpp>
#include <omp.h>

#include "OpenMVS/MVS.h"
using namespace MVS;


void UAVDenseProcess::UAVDP_MVSProcInitialize() {

    //密集点云输出文件
    OPTDENSE::init();
    OPTDENSE::update();
    OPTDENSE::nResolutionLevel = 2;
    OPTDENSE::nMinResolution = 640;
    OPTDENSE::nNumViews = 4;
    OPTDENSE::nMinViewsFuse = 3;
    OPTDENSE::nEstimateColors = 1;
    OPTDENSE::nEstimateNormals = 0;

    // initialize global options
    Process::setCurrentProcessPriority((Process::Priority)-1);
#ifdef _USE_OPENMP
    omp_set_num_threads(2);
#endif
}

void UAVDenseProcess::UAVDP_MVSProc() {
    //初始化
    string export_mvs = stlplus::create_filespec(_info_._g_point_cloud_dir,"trans",".mvs");
    string export_ply = stlplus::create_filespec(_info_._g_point_cloud_dir,"dense",".ply");

    UAVDP_MVSProcInitialize();
    Scene scene(2);

    // load and estimate a dense point-cloud
    if (!scene.Load(MAKE_PATH_SAFE(export_mvs)))
        return ;
    if (scene.pointcloud.IsEmpty()) {
        VERBOSE("error: empty initial point-cloud");
        return ;
    }
    if ((ARCHIVE_TYPE)2 != ARCHIVE_MVS) {
        TD_TIMER_START();
        if (!scene.DenseReconstruction())
            return ;
        VERBOSE("Densifying point-cloud completed: %u points (%s)", scene.pointcloud.points.GetSize(), TD_TIMER_GET_FMT().c_str());
    }

    // save the final mesh
    const String baseFileName(MAKE_PATH_SAFE(Util::getFullFileName(export_ply)));
    scene.Save(baseFileName+_T(".mvs"), (ARCHIVE_TYPE)2);
    scene.pointcloud.Save(baseFileName+_T(".ply"));
#if TD_VERBOSE != TD_VERBOSE_OFF
    if (VERBOSITY_LEVEL > 2)
        scene.ExportCamerasMLP(baseFileName+_T(".mlp"), baseFileName+_T(".ply"));
#endif
}
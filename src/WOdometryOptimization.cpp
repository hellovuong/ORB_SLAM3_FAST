//
// Created by vuong on 2/20/22.
//

#include "WOdometryOptimization.h"
void ORB_SLAM3::WOdometryOptimization::OnlineExtrinsic(
    const vector<KeyFrame*>& vpKFs, Sophus::SE3f& Extrinc) {
  int its = 4;
  size_t maxKFid = 0;

  g2o::SparseOptimizer optimizer;
  g2o::BlockSolverX::LinearSolverType* linearSolver;

  linearSolver =
      new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

  auto* solver_ptr = new g2o::BlockSolverX(linearSolver);

  auto* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(false);

  for (auto* pKFi : vpKFs) {
    auto* VP = new VertexPose(pKFi);
    if (pKFi->mnId > maxKFid) maxKFid = pKFi->mnId;
    VP->setId(static_cast<int>(pKFi->mnId));
    VP->setFixed(true);
    optimizer.addVertex(VP);
  }

  auto* VExtr = new g2o::VertexSE3Expmap();
  Sophus::SE3d Tbo = Extrinc.cast<double>();
  VExtr->setEstimate(g2o::SE3Quat(Tbo.unit_quaternion(), Tbo.translation()));
  VExtr->setId(static_cast<int>(maxKFid + 1));
  VExtr->setFixed(false);
  optimizer.addVertex(VExtr);

  for (size_t i = 0; i < vpKFs.size() - 1; i++) {
    KeyFrame* pKFi = vpKFs[i];

    g2o::HyperGraph::Vertex* VP1 =
        optimizer.vertex(static_cast<int>(pKFi->mPrevKF->mnId));
    g2o::HyperGraph::Vertex* VP2 =
        optimizer.vertex(static_cast<int>(pKFi->mnId));
    g2o::HyperGraph::Vertex* VExtr_ =
        optimizer.vertex(static_cast<int>(maxKFid + 1));
    auto* eo = new EdgeWOdometryExtr(pKFi->mpOdomPreintegrated);

    eo->setVertex(0, static_cast<g2o::OptimizableGraph::Vertex*>(VP1));
    eo->setVertex(1, static_cast<g2o::OptimizableGraph::Vertex*>(VP2));
    eo->setVertex(2, static_cast<g2o::OptimizableGraph::Vertex*>(VExtr_));
    optimizer.addEdge(eo);
  }
  optimizer.initializeOptimization();
  optimizer.optimize(its);

  Extrinc = Sophus::SE3f(VExtr->estimate().rotation().cast<float>(),
                         VExtr->estimate().translation().cast<float>());
  for (auto* pKFi : vpKFs) {
    pKFi->mpOdomPreintegrated->Tbo = Extrinc;
  }
}
void ORB_SLAM3::WOdometryOptimization::FullInertialBANew(
    ORB_SLAM3::Map* pMap, int its, bool bFixLocal, unsigned long nLoopId,
    bool* pbStopFlag, bool bInit, float priorG, float priorA,
    Eigen::VectorXd* vSingVal, bool* bHess,
    PlanarConstraint* pPlanarConstraint) {
  long unsigned int maxKFid = pMap->GetMaxKFid();
  size_t maxId = 0;
  const vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
  const vector<MapPoint*> vpMPs = pMap->GetAllMapPoints();

  g2o::SparseOptimizer optimizer;
  g2o::BlockSolverX::LinearSolverType* linearSolver;

  linearSolver =
      new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

  auto* solver_ptr = new g2o::BlockSolverX(linearSolver);

  auto* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  solver->setUserLambdaInit(1e-5);
  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(false);

  if (pbStopFlag) optimizer.setForceStopFlag(pbStopFlag);

  int nNonFixed = 0;

  // Set KeyFrame vertices
  KeyFrame* pIncKF = nullptr;
  for (auto pKFi : vpKFs) {
    if (pKFi->mnId > maxKFid) continue;
    auto* VP = new VertexPose(pKFi);
    VP->setId((int)(pKFi->mnId));
    if (maxId < pKFi->mnId) maxId = pKFi->mnId;
    pIncKF = pKFi;
    bool bFixed = false;
    if (bFixLocal) {
      bFixed = (pKFi->mnBALocalForKF >= (maxKFid - 1)) ||
               (pKFi->mnBAFixedForKF >= (maxKFid - 1));
      if (!bFixed) nNonFixed++;
      VP->setFixed(bFixed);
    }
    optimizer.addVertex(VP);

    if (pKFi->bImu) {
      auto* VV = new VertexVelocity(pKFi);
      VV->setId((int)(maxKFid + 3 * (pKFi->mnId) + 1));
      VV->setFixed(bFixed);
      if (maxId < (maxKFid + 3 * (pKFi->mnId) + 1)) maxId = pKFi->mnId;
      optimizer.addVertex(VV);
      if (!bInit) {
        auto* VG = new VertexGyroBias(pKFi);
        VG->setId((int)(maxKFid + 3 * (pKFi->mnId) + 2));
        VG->setFixed(bFixed);
        optimizer.addVertex(VG);
        auto* VA = new VertexAccBias(pKFi);
        VA->setId((int)(maxKFid + 3 * (pKFi->mnId) + 3));
        VA->setFixed(bFixed);
        optimizer.addVertex(VA);
        if (maxId < (maxKFid + 3 * (pKFi->mnId) + 3))
          maxId = (maxKFid + 3 * (pKFi->mnId) + 3);
      }
    }
  }

  if (bInit) {
    auto* VG = new VertexGyroBias(pIncKF);
    VG->setId((int)(4 * maxKFid + 2));
    VG->setFixed(false);
    optimizer.addVertex(VG);
    auto* VA = new VertexAccBias(pIncKF);
    VA->setId((int)(4 * maxKFid + 3));
    VA->setFixed(false);
    optimizer.addVertex(VA);
    if (maxId < (4 * maxKFid + 3)) maxId = 4 * maxKFid + 3;
  }

  if (bFixLocal && nNonFixed < 3) return;

  auto* VPlanar = new VertexPlanar();
  if (pPlanarConstraint) {
    VPlanar->setId((int)maxId + 1);
    VPlanar->setFixed(false);
    VPlanar->setEstimate(pPlanarConstraint);
    optimizer.addVertex(VPlanar);
  }

  // IMU links
  for (auto pKFi : vpKFs) {
    if (!pKFi->mPrevKF) {
      Verbose::PrintMess("NOT INERTIAL LINK TO PREVIOUS FRAME!",
                         Verbose::VERBOSITY_NORMAL);
      continue;
    }

    if (pKFi->mPrevKF && pKFi->mnId <= maxKFid) {
      if (pKFi->isBad() || pKFi->mPrevKF->mnId > maxKFid) continue;
      if (pKFi->bImu && pKFi->mPrevKF->bImu) {
        pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());
        g2o::HyperGraph::Vertex* VP1 =
            optimizer.vertex((int)(pKFi->mPrevKF->mnId));
        g2o::HyperGraph::Vertex* VV1 =
            optimizer.vertex((int)(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 1));

        g2o::HyperGraph::Vertex* VG1;
        g2o::HyperGraph::Vertex* VA1;

        g2o::HyperGraph::Vertex* VP2 = optimizer.vertex((int)(pKFi->mnId));
        g2o::HyperGraph::Vertex* VV2 =
            optimizer.vertex((int)(maxKFid + 3 * (pKFi->mnId) + 1));

        if (!bInit) {
          g2o::HyperGraph::Vertex* VG2;
          g2o::HyperGraph::Vertex* VA2;

          VG1 =
              optimizer.vertex((int)(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 2));
          VA1 =
              optimizer.vertex((int)(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 3));

          VG2 = optimizer.vertex((int)(maxKFid + 3 * (pKFi->mnId) + 2));
          VA2 = optimizer.vertex((int)(maxKFid + 3 * (pKFi->mnId) + 3));

          if (!VP1 || !VV1 || !VG1 || !VA1 || !VP2 || !VV2 || !VG2 || !VA2) {
            cout << "Error" << VP1 << ", " << VV1 << ", " << VG1 << ", " << VA1
                 << ", " << VP2 << ", " << VV2 << ", " << VG2 << ", " << VA2
                 << endl;
            continue;
          }
          auto* egr = new EdgeGyroRW();
          egr->setVertex(0, VG1);
          egr->setVertex(1, VG2);
          Eigen::Matrix3d InfoG = pKFi->mpImuPreintegrated->C.block<3, 3>(9, 9)
                                      .cast<double>()
                                      .inverse();
          egr->setInformation(InfoG);
          egr->computeError();
          optimizer.addEdge(egr);

          auto* ear = new EdgeAccRW();
          ear->setVertex(0, VA1);
          ear->setVertex(1, VA2);
          Eigen::Matrix3d InfoA =
              pKFi->mpImuPreintegrated->C.block<3, 3>(12, 12)
                  .cast<double>()
                  .inverse();
          ear->setInformation(InfoA);
          ear->computeError();
          optimizer.addEdge(ear);

        } else {
          VG1 = optimizer.vertex((int)(4 * maxKFid + 2));
          VA1 = optimizer.vertex((int)(4 * maxKFid + 3));
          if (!VP1 || !VV1 || !VG1 || !VA1 || !VP2 || !VV2) {
            cout << "Error" << VP1 << ", " << VV1 << ", " << VG1 << ", " << VA1
                 << ", " << VP2 << ", " << VV2 << endl;
            continue;
          }
        }

        auto* ei = new EdgeInertial(pKFi->mpImuPreintegrated);
        ei->setVertex(0, static_cast<g2o::OptimizableGraph::Vertex*>(VP1));
        ei->setVertex(1, static_cast<g2o::OptimizableGraph::Vertex*>(VV1));
        ei->setVertex(2, static_cast<g2o::OptimizableGraph::Vertex*>(VG1));
        ei->setVertex(3, static_cast<g2o::OptimizableGraph::Vertex*>(VA1));
        ei->setVertex(4, static_cast<g2o::OptimizableGraph::Vertex*>(VP2));
        ei->setVertex(5, static_cast<g2o::OptimizableGraph::Vertex*>(VV2));

        auto* rki = new g2o::RobustKernelHuber;
        ei->setRobustKernel(rki);
        rki->setDelta(sqrt(16.92));

        optimizer.addEdge(ei);

        if (pKFi->mpOdomPreintegrated) {
          auto* eo = new EdgeWOdometry(pKFi->mpOdomPreintegrated);
          eo->setVertex(0, static_cast<g2o::OptimizableGraph::Vertex*>(VP1));
          eo->setVertex(1, static_cast<g2o::OptimizableGraph::Vertex*>(VP2));
          optimizer.addEdge(eo);
        }
        if (pPlanarConstraint) {
          auto* ep = new EdgePlane();
          ep->Tbo = pKFi->mpOdomPreintegrated->Tbo.cast<double>();
          ep->setVertex(0, static_cast<g2o::OptimizableGraph::Vertex*>(VP2));
          ep->setVertex(1,
                        static_cast<g2o::OptimizableGraph::Vertex*>(VPlanar));
          //          ep->setInformation(
          //              pKFi->mpOdomPreintegrated->Cov1.cast<double>().inverse());
          //          optimizer.addEdge(ep);
        }
      } else
        cout << pKFi->mnId << " or " << pKFi->mPrevKF->mnId << " no imu"
             << endl;
    }
  }

  if (bInit) {
    g2o::HyperGraph::Vertex* VG = optimizer.vertex((int)(4 * maxKFid + 2));
    g2o::HyperGraph::Vertex* VA = optimizer.vertex((int)(4 * maxKFid + 3));

    // Add prior to comon biases
    Eigen::Vector3f bprior;
    bprior.setZero();

    auto* epa = new EdgePriorAcc(bprior);
    epa->setVertex(0, static_cast<g2o::OptimizableGraph::Vertex*>(VA));
    double infoPriorA = priorA;  //
    epa->setInformation(infoPriorA * Eigen::Matrix3d::Identity());
    optimizer.addEdge(epa);

    auto* epg = new EdgePriorGyro(bprior);
    epg->setVertex(0, static_cast<g2o::OptimizableGraph::Vertex*>(VG));
    double infoPriorG = priorG;  //
    epg->setInformation(infoPriorG * Eigen::Matrix3d::Identity());
    optimizer.addEdge(epg);
  }

  const double thHuberMono = sqrt(5.991);
  const double thHuberStereo = sqrt(7.815);

  const unsigned long iniMPid = maxKFid * 5;

  vector<bool> vbNotIncludedMP(vpMPs.size(), false);

  for (size_t i = 0; i < vpMPs.size(); i++) {
    MapPoint* pMP = vpMPs[i];
    auto* vPoint = new g2o::VertexSBAPointXYZ();
    vPoint->setEstimate(pMP->GetWorldPos().cast<double>());
    unsigned long id = pMP->mnId + iniMPid + 1;
    vPoint->setId((int)(id));
    vPoint->setMarginalized(true);
    optimizer.addVertex(vPoint);

    const map<KeyFrame*, tuple<int, int>> observations = pMP->GetObservations();

    bool bAllFixed = true;

    // Set edges
    for (const auto& observation : observations) {
      KeyFrame* pKFi = observation.first;

      if (pKFi->mnId > maxKFid) continue;

      if (!pKFi->isBad()) {
        const int leftIndex = get<0>(observation.second);
        cv::KeyPoint kpUn;

        if (leftIndex != -1 && pKFi->mvuRight[get<0>(observation.second)] <
                                   0)  // Monocular observation
        {
          kpUn = pKFi->mvKeysUn[leftIndex];
          Eigen::Matrix<double, 2, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y;

          auto* e = new EdgeMono(0);

          auto* VP = static_cast<g2o::OptimizableGraph::Vertex*>(
              optimizer.vertex((int)(pKFi->mnId)));
          if (bAllFixed)
            if (!VP->fixed()) bAllFixed = false;

          e->setVertex(0, static_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex((int)(id))));
          e->setVertex(1, VP);
          e->setMeasurement(obs);
          const float invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];

          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          auto* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberMono);

          optimizer.addEdge(e);
        } else if (leftIndex != -1 &&
                   pKFi->mvuRight[leftIndex] >= 0)  // stereo observation
        {
          kpUn = pKFi->mvKeysUn[leftIndex];
          const float kp_ur = pKFi->mvuRight[leftIndex];
          Eigen::Matrix<double, 3, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

          auto* e = new EdgeStereo(0);

          auto* VP = static_cast<g2o::OptimizableGraph::Vertex*>(
              optimizer.vertex((int)(pKFi->mnId)));
          if (bAllFixed)
            if (!VP->fixed()) bAllFixed = false;

          e->setVertex(0, static_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex((int)(id))));
          e->setVertex(1, VP);
          e->setMeasurement(obs);
          const float invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];

          e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

          auto* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberStereo);

          optimizer.addEdge(e);
        }

        if (pKFi->mpCamera2) {  // Monocular right observation
          int rightIndex = get<1>(observation.second);

          if (rightIndex != -1 && rightIndex < (int)pKFi->mvKeysRight.size()) {
            rightIndex -= pKFi->NLeft;

            Eigen::Matrix<double, 2, 1> obs;
            kpUn = pKFi->mvKeysRight[rightIndex];
            obs << kpUn.pt.x, kpUn.pt.y;

            auto* e = new EdgeMono(1);

            auto* VP = static_cast<g2o::OptimizableGraph::Vertex*>(
                optimizer.vertex((int)(pKFi->mnId)));
            if (bAllFixed)
              if (!VP->fixed()) bAllFixed = false;

            e->setVertex(0, static_cast<g2o::OptimizableGraph::Vertex*>(
                                optimizer.vertex((int)(id))));
            e->setVertex(1, VP);
            e->setMeasurement(obs);
            const float invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
            e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

            auto* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(thHuberMono);

            optimizer.addEdge(e);
          }
        }
      }
    }

    if (bAllFixed) {
      optimizer.removeVertex(vPoint);
      vbNotIncludedMP[i] = true;
    }
  }

  if (pbStopFlag)
    if (*pbStopFlag) return;

  optimizer.initializeOptimization();
  optimizer.optimize(its);

  // Recover optimized data
  // Keyframes
  for (auto pKFi : vpKFs) {
    if (pKFi->mnId > maxKFid) continue;
    auto* VP = static_cast<VertexPose*>(optimizer.vertex((int)(pKFi->mnId)));
    if (nLoopId == 0) {
      Sophus::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(),
                       VP->estimate().tcw[0].cast<float>());
      pKFi->SetPose(Tcw);
    } else {
      pKFi->mTcwGBA = Sophus::SE3f(VP->estimate().Rcw[0].cast<float>(),
                                   VP->estimate().tcw[0].cast<float>());
      pKFi->mnBAGlobalForKF = nLoopId;
    }
    if (pKFi->bImu) {
      auto* VV = static_cast<VertexVelocity*>(
          optimizer.vertex((int)(maxKFid + 3 * (pKFi->mnId) + 1)));
      if (nLoopId == 0) {
        pKFi->SetVelocity(VV->estimate().cast<float>());
      } else {
        pKFi->mVwbGBA = VV->estimate().cast<float>();
      }

      VertexGyroBias* VG;
      VertexAccBias* VA;
      if (!bInit) {
        VG = static_cast<VertexGyroBias*>(
            optimizer.vertex((int)(maxKFid + 3 * (pKFi->mnId) + 2)));
        VA = static_cast<VertexAccBias*>(
            optimizer.vertex((int)(maxKFid + 3 * (pKFi->mnId) + 3)));
      } else {
        VG = static_cast<VertexGyroBias*>(
            optimizer.vertex((int)(4 * maxKFid + 2)));
        VA = static_cast<VertexAccBias*>(
            optimizer.vertex((int)(4 * maxKFid + 3)));
      }

      Vector6d vb;
      vb << VG->estimate(), VA->estimate();
      IMU::Bias b((float)vb[3], (float)vb[4], (float)vb[5], (float)vb[0],
                  (float)vb[1], (float)vb[2]);
      if (nLoopId == 0) {
        pKFi->SetNewBias(b);
      } else {
        pKFi->mBiasGBA = b;
      }
    }
  }

  // Points
  for (size_t i = 0; i < vpMPs.size(); i++) {
    if (vbNotIncludedMP[i]) continue;

    MapPoint* pMP = vpMPs[i];
    auto* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(
        optimizer.vertex((int)(pMP->mnId + iniMPid + 1)));

    if (nLoopId == 0) {
      pMP->SetWorldPos(vPoint->estimate().cast<float>());
      pMP->UpdateNormalAndDepth();
    } else {
      pMP->mPosGBA = vPoint->estimate().cast<float>();
      pMP->mnBAGlobalForKF = nLoopId;
    }
  }
  // Planar constraints
  if (pPlanarConstraint) {
    auto* VPlanar_Recov =
        static_cast<VertexPlanar*>(optimizer.vertex((int)maxId + 1));
    pPlanarConstraint->setZpw(VPlanar_Recov->estimate().getZpw());
    pPlanarConstraint->setQpw(VPlanar_Recov->estimate().getQpw());
  }
  pMap->IncreaseChangeIndex();
}
void ORB_SLAM3::WOdometryOptimization::LocalInertialBANew(
    ORB_SLAM3::KeyFrame* pKF, bool* pbStopFlag, ORB_SLAM3::Map* pMap,
    int& num_fixedKF, int& num_OptKF, int& num_MPs, int& num_edges, bool bLarge,
    bool bRecInit, PlanarConstraint* pPlanarConstraint) {
  Map* pCurrentMap = pKF->GetMap();

  int maxOpt = 10;
  int opt_it = 20;
  if (bLarge) {
    maxOpt = 25;
    opt_it = 10;
  }
  const int Nd = std::min((int)pCurrentMap->KeyFramesInMap() - 2, maxOpt);
  const unsigned long maxKFid = pKF->mnId;

  vector<KeyFrame*> vpOptimizableKFs;
  const vector<KeyFrame*> vpNeighsKFs = pKF->GetVectorCovisibleKeyFrames();
  list<KeyFrame*> lpOptVisKFs;

  vpOptimizableKFs.reserve(Nd);
  vpOptimizableKFs.push_back(pKF);
  pKF->mnBALocalForKF = pKF->mnId;
  for (int i = 1; i < Nd; i++) {
    if (vpOptimizableKFs.back()->mPrevKF) {
      vpOptimizableKFs.push_back(vpOptimizableKFs.back()->mPrevKF);
      vpOptimizableKFs.back()->mnBALocalForKF = pKF->mnId;
    } else
      break;
  }

  size_t N = vpOptimizableKFs.size();

  // Optimizable points seen by temporal optimizable keyframes
  list<MapPoint*> lLocalMapPoints;
  for (size_t i = 0; i < N; i++) {
    vector<MapPoint*> vpMPs = vpOptimizableKFs[i]->GetMapPointMatches();
    for (auto pMP : vpMPs) {
      if (pMP)
        if (!pMP->isBad())
          if (pMP->mnBALocalForKF != pKF->mnId) {
            lLocalMapPoints.push_back(pMP);
            pMP->mnBALocalForKF = pKF->mnId;
          }
    }
  }

  // Fixed Keyframe: First frame previous KF to optimization window)
  list<KeyFrame*> lFixedKeyFrames;
  if (vpOptimizableKFs.back()->mPrevKF) {
    lFixedKeyFrames.push_back(vpOptimizableKFs.back()->mPrevKF);
    vpOptimizableKFs.back()->mPrevKF->mnBAFixedForKF = pKF->mnId;
  } else {
    vpOptimizableKFs.back()->mnBALocalForKF = 0;
    vpOptimizableKFs.back()->mnBAFixedForKF = pKF->mnId;
    lFixedKeyFrames.push_back(vpOptimizableKFs.back());
    vpOptimizableKFs.pop_back();
  }

  // Optimizable visual KFs
  const int maxCovKF = 0;
  for (auto pKFi : vpNeighsKFs) {
    if (lpOptVisKFs.size() >= maxCovKF) break;

    if (pKFi->mnBALocalForKF == pKF->mnId || pKFi->mnBAFixedForKF == pKF->mnId)
      continue;
    pKFi->mnBALocalForKF = pKF->mnId;
    if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap) {
      lpOptVisKFs.push_back(pKFi);

      vector<MapPoint*> vpMPs = pKFi->GetMapPointMatches();
      for (auto pMP : vpMPs) {
        if (pMP)
          if (!pMP->isBad())
            if (pMP->mnBALocalForKF != pKF->mnId) {
              lLocalMapPoints.push_back(pMP);
              pMP->mnBALocalForKF = pKF->mnId;
            }
      }
    }
  }

  // Fixed KFs which are not covisible optimizable
  const int maxFixKF = 200;

  for (auto& lLocalMapPoint : lLocalMapPoints) {
    map<KeyFrame*, tuple<int, int>> observations =
        lLocalMapPoint->GetObservations();
    for (auto& observation : observations) {
      KeyFrame* pKFi = observation.first;

      if (pKFi->mnBALocalForKF != pKF->mnId &&
          pKFi->mnBAFixedForKF != pKF->mnId) {
        pKFi->mnBAFixedForKF = pKF->mnId;
        if (!pKFi->isBad()) {
          lFixedKeyFrames.push_back(pKFi);
          break;
        }
      }
    }
    if (lFixedKeyFrames.size() >= maxFixKF) break;
  }
  // Setup optimizer
  g2o::SparseOptimizer optimizer;
  g2o::BlockSolverX::LinearSolverType* linearSolver;
  linearSolver =
      new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

  auto* solver_ptr = new g2o::BlockSolverX(linearSolver);

  if (bLarge) {
    auto* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    solver->setUserLambdaInit(
        1e-2);  // to avoid iterating for finding optimal lambda
    optimizer.setAlgorithm(solver);
  } else {
    auto* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    solver->setUserLambdaInit(1e0);
    optimizer.setAlgorithm(solver);
  }

  // Set Local temporal KeyFrame vertices
  N = vpOptimizableKFs.size();
  for (size_t i = 0; i < N; i++) {
    KeyFrame* pKFi = vpOptimizableKFs[i];

    auto* VP = new VertexPose(pKFi);
    VP->setId((int)(pKFi->mnId));
    VP->setFixed(false);
    optimizer.addVertex(VP);

    if (pKFi->bImu) {
      auto* VV = new VertexVelocity(pKFi);
      VV->setId((int)(maxKFid + 3 * (pKFi->mnId) + 1));
      VV->setFixed(false);
      optimizer.addVertex(VV);
      auto* VG = new VertexGyroBias(pKFi);
      VG->setId((int)(maxKFid + 3 * (pKFi->mnId) + 2));
      VG->setFixed(false);
      optimizer.addVertex(VG);
      auto* VA = new VertexAccBias(pKFi);
      VA->setId((int)(maxKFid + 3 * (pKFi->mnId) + 3));
      VA->setFixed(false);
      optimizer.addVertex(VA);
    }
  }

  // Set Local visual KeyFrame vertices
  for (auto pKFi : lpOptVisKFs) {
    auto* VP = new VertexPose(pKFi);
    VP->setId((int)(pKFi->mnId));
    VP->setFixed(false);
    optimizer.addVertex(VP);
  }

  // Set Fixed KeyFrame vertices
  for (auto pKFi : lFixedKeyFrames) {
    auto* VP = new VertexPose(pKFi);
    VP->setId((int)pKFi->mnId);
    VP->setFixed(true);
    optimizer.addVertex(VP);

    if (pKFi->bImu)  // This should be done only for keyframe just before
                     // temporal window
    {
      auto* VV = new VertexVelocity(pKFi);
      VV->setId((int)(maxKFid + 3 * (pKFi->mnId) + 1));
      VV->setFixed(true);
      optimizer.addVertex(VV);
      auto* VG = new VertexGyroBias(pKFi);
      VG->setId((int)(maxKFid + 3 * (pKFi->mnId) + 2));
      VG->setFixed(true);
      optimizer.addVertex(VG);
      auto* VA = new VertexAccBias(pKFi);
      VA->setId((int)(maxKFid + 3 * (pKFi->mnId) + 3));
      VA->setFixed(true);
      optimizer.addVertex(VA);
    }
  }

  auto* VPlanar = new VertexPlanar();
  if (pPlanarConstraint) {
    VPlanar->setId((int)maxKFid * 5);
    VPlanar->setEstimate(pPlanarConstraint);
    VPlanar->setFixed(false);
    optimizer.addVertex(VPlanar);
  }

  // Create intertial constraints
  vector<EdgeInertial*> vei(N, (EdgeInertial*)nullptr);
  vector<EdgeGyroRW*> vegr(N, (EdgeGyroRW*)nullptr);
  vector<EdgeAccRW*> vear(N, (EdgeAccRW*)nullptr);
  vector<EdgeWOdometry*> veo(N, (EdgeWOdometry*)nullptr);

  for (size_t i = 0; i < N; i++) {
    KeyFrame* pKFi = vpOptimizableKFs[i];

    if (!pKFi->mPrevKF) {
      cout << "NOT INERTIAL LINK TO PREVIOUS FRAME!!!!" << endl;
      continue;
    }
    if (pKFi->bImu && pKFi->mPrevKF->bImu && pKFi->mpImuPreintegrated) {
      pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());
      g2o::HyperGraph::Vertex* VP1 =
          optimizer.vertex((int)(pKFi->mPrevKF->mnId));
      g2o::HyperGraph::Vertex* VV1 =
          optimizer.vertex((int)(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 1));
      g2o::HyperGraph::Vertex* VG1 =
          optimizer.vertex((int)(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 2));
      g2o::HyperGraph::Vertex* VA1 =
          optimizer.vertex((int)(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 3));
      g2o::HyperGraph::Vertex* VP2 = optimizer.vertex((int)(pKFi->mnId));
      g2o::HyperGraph::Vertex* VV2 =
          optimizer.vertex((int)(maxKFid + 3 * (pKFi->mnId) + 1));
      g2o::HyperGraph::Vertex* VG2 =
          optimizer.vertex((int)(maxKFid + 3 * (pKFi->mnId) + 2));
      g2o::HyperGraph::Vertex* VA2 =
          optimizer.vertex((int)(maxKFid + 3 * (pKFi->mnId) + 3));

      if (!VP1 || !VV1 || !VG1 || !VA1 || !VP2 || !VV2 || !VG2 || !VA2) {
        cerr << "Error " << VP1 << ", " << VV1 << ", " << VG1 << ", " << VA1
             << ", " << VP2 << ", " << VV2 << ", " << VG2 << ", " << VA2
             << endl;
        continue;
      }

      vei[i] = new EdgeInertial(pKFi->mpImuPreintegrated);

      vei[i]->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP1));
      vei[i]->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV1));
      vei[i]->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG1));
      vei[i]->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA1));
      vei[i]->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP2));
      vei[i]->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV2));

      if (i == N - 1 || bRecInit) {
        // All inertial residuals are included without robust cost function, but
        // not that one linking the last optimizable keyframe inside of the
        // local window and the first fixed keyframe out. The information matrix
        // for this measurement is also downweighted. This is done to avoid
        // accumulating error due to fixing variables.
        auto* rki = new g2o::RobustKernelHuber;
        vei[i]->setRobustKernel(rki);
        if (i == N - 1) vei[i]->setInformation(vei[i]->information() * 1e-2);
        rki->setDelta(sqrt(16.92));
      }
      optimizer.addEdge(vei[i]);

      vegr[i] = new EdgeGyroRW();
      vegr[i]->setVertex(0, VG1);
      vegr[i]->setVertex(1, VG2);
      Eigen::Matrix3d InfoG = pKFi->mpImuPreintegrated->C.block<3, 3>(9, 9)
                                  .cast<double>()
                                  .inverse();
      vegr[i]->setInformation(InfoG);
      optimizer.addEdge(vegr[i]);

      vear[i] = new EdgeAccRW();
      vear[i]->setVertex(0, VA1);
      vear[i]->setVertex(1, VA2);
      Eigen::Matrix3d InfoA = pKFi->mpImuPreintegrated->C.block<3, 3>(12, 12)
                                  .cast<double>()
                                  .inverse();
      vear[i]->setInformation(InfoA);
      optimizer.addEdge(vear[i]);

      if (pKFi->mpOdomPreintegrated) {
        veo[i] = new EdgeWOdometry(pKFi->mpOdomPreintegrated);
        veo[i]->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP1));
        veo[i]->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP2));
        optimizer.addEdge(veo[i]);
      }

      if (pPlanarConstraint) {
        auto* ep = new EdgePlane();
        ep->Tbo = pKFi->mpOdomPreintegrated->Tbo.cast<double>();
        ep->setVertex(0, VP2);
        ep->setVertex(1, VPlanar);
        optimizer.addEdge(ep);
      }

    } else
      cout << "ERROR building inertial edge" << endl;
  }

  // Set MapPoint vertices
  const size_t nExpectedSize =
      (N + lFixedKeyFrames.size()) * lLocalMapPoints.size();

  // Mono
  vector<EdgeMono*> vpEdgesMono;
  vpEdgesMono.reserve(nExpectedSize);

  vector<KeyFrame*> vpEdgeKFMono;
  vpEdgeKFMono.reserve(nExpectedSize);

  vector<MapPoint*> vpMapPointEdgeMono;
  vpMapPointEdgeMono.reserve(nExpectedSize);

  // Stereo
  vector<EdgeStereo*> vpEdgesStereo;
  vpEdgesStereo.reserve(nExpectedSize);

  vector<KeyFrame*> vpEdgeKFStereo;
  vpEdgeKFStereo.reserve(nExpectedSize);

  vector<MapPoint*> vpMapPointEdgeStereo;
  vpMapPointEdgeStereo.reserve(nExpectedSize);

  const double thHuberMono = sqrt(5.991);
  const float chi2Mono2 = 5.991;
  const double thHuberStereo = sqrt(7.815);
  const float chi2Stereo2 = 7.815;

  const unsigned long iniMPid = maxKFid * 5 + 1;

  map<int, int> mVisEdges;
  for (size_t i = 0; i < N; i++) {
    KeyFrame* pKFi = vpOptimizableKFs[i];
    mVisEdges[(int)pKFi->mnId] = 0;
  }
  for (auto& lFixedKeyFrame : lFixedKeyFrames) {
    mVisEdges[(int)lFixedKeyFrame->mnId] = 0;
  }

  for (auto pMP : lLocalMapPoints) {
    auto* vPoint = new g2o::VertexSBAPointXYZ();
    vPoint->setEstimate(pMP->GetWorldPos().cast<double>());

    unsigned long id = pMP->mnId + iniMPid + 1;
    vPoint->setId((int)id);
    vPoint->setMarginalized(true);
    optimizer.addVertex(vPoint);
    const map<KeyFrame*, tuple<int, int>> observations = pMP->GetObservations();

    // Create visual constraints
    for (const auto& observation : observations) {
      KeyFrame* pKFi = observation.first;

      if (pKFi->mnBALocalForKF != pKF->mnId &&
          pKFi->mnBAFixedForKF != pKF->mnId)
        continue;

      if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap) {
        const int leftIndex = get<0>(observation.second);

        cv::KeyPoint kpUn;

        // Monocular left observation
        if (leftIndex != -1 && pKFi->mvuRight[leftIndex] < 0) {
          mVisEdges[(int)pKFi->mnId]++;

          kpUn = pKFi->mvKeysUn[leftIndex];
          Eigen::Matrix<double, 2, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y;

          auto* e = new EdgeMono(0);

          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex((int)id)));
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex((int)pKFi->mnId)));
          e->setMeasurement(obs);

          // Add here uncerteinty
          const float unc2 = pKFi->mpCamera->uncertainty2(obs);

          const float& invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave] / unc2;
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          auto* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberMono);

          optimizer.addEdge(e);
          vpEdgesMono.push_back(e);
          vpEdgeKFMono.push_back(pKFi);
          vpMapPointEdgeMono.push_back(pMP);
        }
        // Stereo-observation
        else if (leftIndex != -1)  // Stereo observation
        {
          kpUn = pKFi->mvKeysUn[leftIndex];
          mVisEdges[(int)pKFi->mnId]++;

          const float kp_ur = pKFi->mvuRight[leftIndex];
          Eigen::Matrix<double, 3, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

          auto* e = new EdgeStereo(0);

          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex((int)id)));
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertex((int)pKFi->mnId)));
          e->setMeasurement(obs);

          // Add here uncerteinty
          const float unc2 = pKFi->mpCamera->uncertainty2(obs.head(2));

          const float& invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave] / unc2;
          e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

          auto* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberStereo);

          optimizer.addEdge(e);
          vpEdgesStereo.push_back(e);
          vpEdgeKFStereo.push_back(pKFi);
          vpMapPointEdgeStereo.push_back(pMP);
        }

        // Monocular right observation
        if (pKFi->mpCamera2) {
          int rightIndex = get<1>(observation.second);

          if (rightIndex != -1) {
            rightIndex -= pKFi->NLeft;
            mVisEdges[(int)pKFi->mnId]++;

            Eigen::Matrix<double, 2, 1> obs;
            cv::KeyPoint kp = pKFi->mvKeysRight[rightIndex];
            obs << kp.pt.x, kp.pt.y;

            auto* e = new EdgeMono(1);

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                optimizer.vertex((int)id)));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                                optimizer.vertex((int)pKFi->mnId)));
            e->setMeasurement(obs);

            // Add here uncerteinty
            const float unc2 = pKFi->mpCamera->uncertainty2(obs);

            const float& invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave] / unc2;
            e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

            auto* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(thHuberMono);

            optimizer.addEdge(e);
            vpEdgesMono.push_back(e);
            vpEdgeKFMono.push_back(pKFi);
            vpMapPointEdgeMono.push_back(pMP);
          }
        }
      }
    }
  }

  // cout << "Total map points: " << lLocalMapPoints.size() << endl;
  for (auto mit = mVisEdges.begin(), mend = mVisEdges.end(); mit != mend;
       mit++) {
    //        std::cout << "Why we get inside this assert?" << std::endl;
    //        assert(mit->second>=3);
  }

  optimizer.initializeOptimization();
  optimizer.computeActiveErrors();
  double err = optimizer.activeRobustChi2();
  optimizer.optimize(opt_it);  // Originally to 2
  double err_end = optimizer.activeRobustChi2();
  if (pbStopFlag) optimizer.setForceStopFlag(pbStopFlag);

  vector<pair<KeyFrame*, MapPoint*>> vToErase;
  vToErase.reserve(vpEdgesMono.size() + vpEdgesStereo.size());

  // Check inlier observations
  // Mono
  for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
    EdgeMono* e = vpEdgesMono[i];
    MapPoint* pMP = vpMapPointEdgeMono[i];
    bool bClose = pMP->mTrackDepth < 10.f;

    if (pMP->isBad()) continue;

    if ((e->chi2() > chi2Mono2 && !bClose) ||
        (e->chi2() > 1.5f * chi2Mono2 && bClose) || !e->isDepthPositive()) {
      KeyFrame* pKFi = vpEdgeKFMono[i];
      vToErase.push_back(make_pair(pKFi, pMP));
    }
  }

  // Stereo
  for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
    EdgeStereo* e = vpEdgesStereo[i];
    MapPoint* pMP = vpMapPointEdgeStereo[i];

    if (pMP->isBad()) continue;

    if (e->chi2() > chi2Stereo2) {
      KeyFrame* pKFi = vpEdgeKFStereo[i];
      vToErase.push_back(make_pair(pKFi, pMP));
    }
  }

  // Get Map Mutex and erase outliers
  unique_lock<mutex> lock(pMap->mMutexMapUpdate);

  // TODO: Some convergence problems have been detected here
  if ((2 * err < err_end || isnan(err) || isnan(err_end)) && !bLarge)  // bGN)
  {
    cout << "FAIL LOCAL-INERTIAL BA!!!!" << endl;
    return;
  }

  if (!vToErase.empty()) {
    for (auto& i : vToErase) {
      KeyFrame* pKFi = i.first;
      MapPoint* pMPi = i.second;
      pKFi->EraseMapPointMatch(pMPi);
      pMPi->EraseObservation(pKFi);
    }
  }

  for (auto& lFixedKeyFrame : lFixedKeyFrames)
    lFixedKeyFrame->mnBAFixedForKF = 0;

  // Recover optimized data
  // Local temporal Keyframes
  N = vpOptimizableKFs.size();
  for (size_t i = 0; i < N; i++) {
    KeyFrame* pKFi = vpOptimizableKFs[i];

    auto* VP = dynamic_cast<VertexPose*>(optimizer.vertex((int)pKFi->mnId));
    Sophus::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(),
                     VP->estimate().tcw[0].cast<float>());
    pKFi->SetPose(Tcw);
    pKFi->mnBALocalForKF = 0;

    if (pKFi->bImu) {
      auto* VV = dynamic_cast<VertexVelocity*>(
          optimizer.vertex((int)(maxKFid + 3 * (pKFi->mnId) + 1)));
      pKFi->SetVelocity(VV->estimate().cast<float>());
      auto* VG = dynamic_cast<VertexGyroBias*>(
          optimizer.vertex((int)(maxKFid + 3 * (pKFi->mnId) + 2)));
      auto* VA = dynamic_cast<VertexAccBias*>(
          optimizer.vertex((int)(maxKFid + 3 * (pKFi->mnId) + 3)));
      Vector6d b;
      b << VG->estimate(), VA->estimate();
      pKFi->SetNewBias(IMU::Bias((float)b[3], (float)b[4], (float)b[5],
                                 (float)b[0], (float)b[1], (float)b[2]));
    }
  }

  // Local visual KeyFrame
  for (auto pKFi : lpOptVisKFs) {
    auto* VP = dynamic_cast<VertexPose*>(optimizer.vertex((int)pKFi->mnId));
    Sophus::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(),
                     VP->estimate().tcw[0].cast<float>());
    pKFi->SetPose(Tcw);
    pKFi->mnBALocalForKF = 0;
  }

  // Points
  for (auto pMP : lLocalMapPoints) {
    auto* vPoint = dynamic_cast<g2o::VertexSBAPointXYZ*>(
        optimizer.vertex((int)(pMP->mnId + iniMPid + 1)));
    pMP->SetWorldPos(vPoint->estimate().cast<float>());
    pMP->UpdateNormalAndDepth();
  }
  if (pPlanarConstraint) {
    auto* vPlanar_Recov =
        static_cast<VertexPlanar*>(optimizer.vertex((int)maxKFid * 5));
    pPlanarConstraint->setQpw(vPlanar_Recov->estimate().getQpw());
    pPlanarConstraint->setZpw(vPlanar_Recov->estimate().getZpw());
  }
  pMap->IncreaseChangeIndex();
}
int ORB_SLAM3::WOdometryOptimization::PoseInertialOptimizationLastKeyFrame(
    ORB_SLAM3::Frame* pFrame, PlanarConstraint* pPlanarConstraint,
    bool bRecInit) {
  g2o::SparseOptimizer optimizer;
  g2o::BlockSolverX::LinearSolverType* linearSolver;

  linearSolver =
      new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

  auto* solver_ptr = new g2o::BlockSolverX(linearSolver);

  auto* solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
  optimizer.setVerbose(false);
  optimizer.setAlgorithm(solver);

  int nInitialMonoCorrespondences = 0;
  int nInitialStereoCorrespondences = 0;
  int nInitialCorrespondences = 0;

  // Set Frame vertex
  auto* VP = new VertexPose(pFrame);
  VP->setId(0);
  VP->setFixed(false);
  optimizer.addVertex(VP);
  auto* VV = new VertexVelocity(pFrame);
  VV->setId(1);
  VV->setFixed(false);
  optimizer.addVertex(VV);
  auto* VG = new VertexGyroBias(pFrame);
  VG->setId(2);
  VG->setFixed(false);
  optimizer.addVertex(VG);
  auto* VA = new VertexAccBias(pFrame);
  VA->setId(3);
  VA->setFixed(false);
  optimizer.addVertex(VA);

  // Set MapPoint vertices
  const int N = pFrame->N;
  const int Nleft = pFrame->Nleft;
  const bool bRight = (Nleft != -1);

  vector<EdgeMonoOnlyPose*> vpEdgesMono;
  vector<EdgeStereoOnlyPose*> vpEdgesStereo;
  vector<size_t> vnIndexEdgeMono;
  vector<size_t> vnIndexEdgeStereo;
  vpEdgesMono.reserve(N);
  vpEdgesStereo.reserve(N);
  vnIndexEdgeMono.reserve(N);
  vnIndexEdgeStereo.reserve(N);

  const double thHuberMono = sqrt(5.991);
  const double thHuberStereo = sqrt(7.815);

  {
    unique_lock<mutex> lock(MapPoint::mGlobalMutex);

    for (int i = 0; i < N; i++) {
      MapPoint* pMP = pFrame->mvpMapPoints[i];
      if (pMP) {
        cv::KeyPoint kpUn;
        // Left monocular observation
        if ((!bRight && pFrame->mvuRight[i] < 0) || i < Nleft) {
          if (i < Nleft)  // pair left-right
            kpUn = pFrame->mvKeys[i];
          else
            kpUn = pFrame->mvKeysUn[i];

          nInitialMonoCorrespondences++;
          pFrame->mvbOutlier[i] = false;

          Eigen::Matrix<double, 2, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y;

          auto* e = new EdgeMonoOnlyPose(pMP->GetWorldPos(), 0);

          e->setVertex(0, VP);
          e->setMeasurement(obs);

          // Add here uncerteinty
          const float unc2 = pFrame->mpCamera->uncertainty2(obs);

          const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          auto* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberMono);

          optimizer.addEdge(e);

          vpEdgesMono.push_back(e);
          vnIndexEdgeMono.push_back(i);
        }
        // Stereo observation
        else if (!bRight) {
          nInitialStereoCorrespondences++;
          pFrame->mvbOutlier[i] = false;

          kpUn = pFrame->mvKeysUn[i];
          const float kp_ur = pFrame->mvuRight[i];
          Eigen::Matrix<double, 3, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

          auto* e = new EdgeStereoOnlyPose(pMP->GetWorldPos());

          e->setVertex(0, VP);
          e->setMeasurement(obs);

          // Add here uncerteinty
          const float unc2 = pFrame->mpCamera->uncertainty2(obs.head(2));

          const float& invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
          e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

          auto* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberStereo);

          optimizer.addEdge(e);

          vpEdgesStereo.push_back(e);
          vnIndexEdgeStereo.push_back(i);
        }

        // Right monocular observation
        if (bRight && i >= Nleft) {
          nInitialMonoCorrespondences++;
          pFrame->mvbOutlier[i] = false;

          kpUn = pFrame->mvKeysRight[i - Nleft];
          Eigen::Matrix<double, 2, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y;

          auto* e = new EdgeMonoOnlyPose(pMP->GetWorldPos(), 1);

          e->setVertex(0, VP);
          e->setMeasurement(obs);

          // Add here uncerteinty
          const float unc2 = pFrame->mpCamera->uncertainty2(obs);

          const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          auto* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberMono);

          optimizer.addEdge(e);

          vpEdgesMono.push_back(e);
          vnIndexEdgeMono.push_back(i);
        }
      }
    }
  }
  nInitialCorrespondences =
      nInitialMonoCorrespondences + nInitialStereoCorrespondences;

  KeyFrame* pKF = pFrame->mpLastKeyFrame;
  auto* VPk = new VertexPose(pKF);
  VPk->setId(4);
  VPk->setFixed(true);
  optimizer.addVertex(VPk);
  auto* VVk = new VertexVelocity(pKF);
  VVk->setId(5);
  VVk->setFixed(true);
  optimizer.addVertex(VVk);
  auto* VGk = new VertexGyroBias(pKF);
  VGk->setId(6);
  VGk->setFixed(true);
  optimizer.addVertex(VGk);
  auto* VAk = new VertexAccBias(pKF);
  VAk->setId(7);
  VAk->setFixed(true);
  optimizer.addVertex(VAk);

  auto* ei = new EdgeInertial(pFrame->mpImuPreintegrated);

  ei->setVertex(0, VPk);
  ei->setVertex(1, VVk);
  ei->setVertex(2, VGk);
  ei->setVertex(3, VAk);
  ei->setVertex(4, VP);
  ei->setVertex(5, VV);
  optimizer.addEdge(ei);
  auto* egr = new EdgeGyroRW();
  egr->setVertex(0, VGk);
  egr->setVertex(1, VG);
  Eigen::Matrix3d InfoG =
      pFrame->mpImuPreintegrated->C.block<3, 3>(9, 9).cast<double>().inverse();
  egr->setInformation(InfoG);
  optimizer.addEdge(egr);

  auto* ear = new EdgeAccRW();
  ear->setVertex(0, VAk);
  ear->setVertex(1, VA);
  Eigen::Matrix3d InfoA = pFrame->mpImuPreintegrated->C.block<3, 3>(12, 12)
                              .cast<double>()
                              .inverse();
  ear->setInformation(InfoA);
  optimizer.addEdge(ear);

  if (pFrame->mpOdomPreintegrated) {
    auto* eo = new EdgeWOdometry(pFrame->mpOdomPreintegrated);
    eo->setVertex(0, VPk);
    eo->setVertex(1, VP);
    optimizer.addEdge(eo);
  }

  if (pPlanarConstraint) {
    auto* VPlanar = new VertexPlanar();
    VPlanar->setEstimate(pPlanarConstraint);
    VPlanar->setFixed(true);
    VPlanar->setId(8);
    optimizer.addVertex(VPlanar);

    auto* ep = new EdgePlane();
    ep->Tbo = pFrame->mpOdomPreintegrated->Tbo.cast<double>();
    ep->setVertex(0, VP);
    ep->setVertex(1, VPlanar);
    optimizer.addEdge(ep);
  }

  // We perform 4 optimizations, after each optimization we classify observation
  // as inlier/outlier At the next optimization, outliers are not included, but
  // at the end they can be classified as inliers again.
  float chi2Mono[4] = {12, 7.5, 5.991, 5.991};
  float chi2Stereo[4] = {15.6, 9.8, 7.815, 7.815};

  int its[4] = {10, 10, 10, 10};

  int nBad = 0;
  int nBadMono = 0;
  int nBadStereo = 0;
  int nInliersMono = 0;
  int nInliersStereo = 0;
  int nInliers = 0;
  for (size_t it = 0; it < 4; it++) {
    optimizer.initializeOptimization(0);
    optimizer.optimize(its[it]);

    nBad = 0;
    nBadMono = 0;
    nBadStereo = 0;
    nInliers = 0;
    nInliersMono = 0;
    nInliersStereo = 0;
    float chi2close = 1.5f * chi2Mono[it];

    // For monocular observations
    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
      EdgeMonoOnlyPose* e = vpEdgesMono[i];

      const size_t idx = vnIndexEdgeMono[i];
      bool bClose = pFrame->mvpMapPoints[idx]->mTrackDepth < 10.f;

      if (pFrame->mvbOutlier[idx]) {
        e->computeError();
      }

      const double chi2 = e->chi2();

      if ((chi2 > chi2Mono[it] && !bClose) || (bClose && chi2 > chi2close) ||
          !e->isDepthPositive()) {
        pFrame->mvbOutlier[idx] = true;
        e->setLevel(1);
        nBadMono++;
      } else {
        pFrame->mvbOutlier[idx] = false;
        e->setLevel(0);
        nInliersMono++;
      }

      if (it == 2) e->setRobustKernel(nullptr);
    }

    for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
      EdgeStereoOnlyPose* e = vpEdgesStereo[i];

      const size_t idx = vnIndexEdgeStereo[i];

      if (pFrame->mvbOutlier[idx]) {
        e->computeError();
      }

      const double chi2 = e->chi2();

      if (chi2 > chi2Stereo[it]) {
        pFrame->mvbOutlier[idx] = true;
        e->setLevel(1);
        nBadStereo++;
      } else {
        pFrame->mvbOutlier[idx] = false;
        e->setLevel(0);
        nInliersStereo++;
      }

      if (it == 2) e->setRobustKernel(nullptr);
    }
    //                      });

    nInliers = nInliersMono + nInliersStereo;
    nBad = nBadMono + nBadStereo;

    if (optimizer.edges().size() < 10) {
      break;
    }
  }

  // If not too much tracks, recover not too bad points
  if ((nInliers < 30) && !bRecInit) {
    nBad = 0;
    const float chi2MonoOut = 18.f;
    const float chi2StereoOut = 24.f;
    EdgeMonoOnlyPose* e1;
    EdgeStereoOnlyPose* e2;
    for (size_t i = 0, iend = vnIndexEdgeMono.size(); i < iend; i++) {
      const size_t idx = vnIndexEdgeMono[i];
      e1 = vpEdgesMono[i];
      e1->computeError();
      if (e1->chi2() < chi2MonoOut)
        pFrame->mvbOutlier[idx] = false;
      else
        nBad++;
    }
    for (size_t i = 0, iend = vnIndexEdgeStereo.size(); i < iend; i++) {
      const size_t idx = vnIndexEdgeStereo[i];
      e2 = vpEdgesStereo[i];
      e2->computeError();
      if (e2->chi2() < chi2StereoOut)
        pFrame->mvbOutlier[idx] = false;
      else
        nBad++;
    }
  }

  // Recover optimized pose, velocity and biases
  pFrame->SetImuPoseVelocity(VP->estimate().Rwb.cast<float>(),
                             VP->estimate().twb.cast<float>(),
                             VV->estimate().cast<float>());
  Vector6d b;
  b << VG->estimate(), VA->estimate();
  pFrame->mImuBias = IMU::Bias((float)b[3], (float)b[4], (float)b[5],
                               (float)b[0], (float)b[1], (float)b[2]);

  // Recover Hessian, marginalize keyframe states and generate new prior for
  // frame
  Eigen::Matrix<double, 15, 15> H;
  H.setZero();

  H.block<9, 9>(0, 0) += ei->GetHessian2();
  H.block<3, 3>(9, 9) += egr->GetHessian2();
  H.block<3, 3>(12, 12) += ear->GetHessian2();

  int tot_in = 0, tot_out = 0;
  for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
    EdgeMonoOnlyPose* e = vpEdgesMono[i];

    const size_t idx = vnIndexEdgeMono[i];

    if (!pFrame->mvbOutlier[idx]) {
      H.block<6, 6>(0, 0) += e->GetHessian();
      tot_in++;
    } else
      tot_out++;
  }

  for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
    EdgeStereoOnlyPose* e = vpEdgesStereo[i];

    const size_t idx = vnIndexEdgeStereo[i];

    if (!pFrame->mvbOutlier[idx]) {
      H.block<6, 6>(0, 0) += e->GetHessian();
      tot_in++;
    } else
      tot_out++;
  }

  pFrame->mpcpi =
      new ConstraintPoseImu(VP->estimate().Rwb, VP->estimate().twb,
                            VV->estimate(), VG->estimate(), VA->estimate(), H);

  return nInitialCorrespondences - nBad;
}
int ORB_SLAM3::WOdometryOptimization::PoseInertialOptimizationLastFrame(
    ORB_SLAM3::Frame* pFrame, ORB_SLAM3::PlanarConstraint* pPlanarConstraint,
    bool bRecInit) {
  g2o::SparseOptimizer optimizer;
  g2o::BlockSolverX::LinearSolverType* linearSolver;

  linearSolver =
      new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

  auto* solver_ptr = new g2o::BlockSolverX(linearSolver);

  auto solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(false);

  int nInitialMonoCorrespondences = 0;
  int nInitialStereoCorrespondences = 0;
  int nInitialCorrespondences = 0;

  // Set Current Frame vertex
  auto* VP = new VertexPose(pFrame);
  VP->setId(0);
  VP->setFixed(false);
  optimizer.addVertex(VP);
  auto* VV = new VertexVelocity(pFrame);
  VV->setId(1);
  VV->setFixed(false);
  optimizer.addVertex(VV);
  auto* VG = new VertexGyroBias(pFrame);
  VG->setId(2);
  VG->setFixed(false);
  optimizer.addVertex(VG);
  auto* VA = new VertexAccBias(pFrame);
  VA->setId(3);
  VA->setFixed(false);
  optimizer.addVertex(VA);

  // Set MapPoint vertices
  const int N = pFrame->N;
  const int Nleft = pFrame->Nleft;
  const bool bRight = (Nleft != -1);

  vector<EdgeMonoOnlyPose*> vpEdgesMono;
  vector<EdgeStereoOnlyPose*> vpEdgesStereo;
  vector<size_t> vnIndexEdgeMono;
  vector<size_t> vnIndexEdgeStereo;
  vpEdgesMono.reserve(N);
  vpEdgesStereo.reserve(N);
  vnIndexEdgeMono.reserve(N);
  vnIndexEdgeStereo.reserve(N);

  const double thHuberMono = sqrt(5.991);
  const double thHuberStereo = sqrt(7.815);

  {
    unique_lock<mutex> lock(MapPoint::mGlobalMutex);

    for (int i = 0; i < N; i++) {
      MapPoint* pMP = pFrame->mvpMapPoints[i];
      if (pMP) {
        cv::KeyPoint kpUn;
        // Left monocular observation
        if ((!bRight && pFrame->mvuRight[i] < 0) || i < Nleft) {
          if (i < Nleft)  // pair left-right
            kpUn = pFrame->mvKeys[i];
          else
            kpUn = pFrame->mvKeysUn[i];

          nInitialMonoCorrespondences++;
          pFrame->mvbOutlier[i] = false;

          Eigen::Matrix<double, 2, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y;

          auto* e = new EdgeMonoOnlyPose(pMP->GetWorldPos(), 0);

          e->setVertex(0, VP);
          e->setMeasurement(obs);

          // Add here uncerteinty
          const float unc2 = pFrame->mpCamera->uncertainty2(obs);

          const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          auto* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberMono);

          optimizer.addEdge(e);

          vpEdgesMono.push_back(e);
          vnIndexEdgeMono.push_back(i);
        }
        // Stereo observation
        else if (!bRight) {
          nInitialStereoCorrespondences++;
          pFrame->mvbOutlier[i] = false;

          kpUn = pFrame->mvKeysUn[i];
          const float kp_ur = pFrame->mvuRight[i];
          Eigen::Matrix<double, 3, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

          auto* e = new EdgeStereoOnlyPose(pMP->GetWorldPos());

          e->setVertex(0, VP);
          e->setMeasurement(obs);

          // Add here uncerteinty
          const float unc2 = pFrame->mpCamera->uncertainty2(obs.head(2));

          const float& invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
          e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

          auto* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberStereo);

          optimizer.addEdge(e);

          vpEdgesStereo.push_back(e);
          vnIndexEdgeStereo.push_back(i);
        }

        // Right monocular observation
        if (bRight && i >= Nleft) {
          nInitialMonoCorrespondences++;
          pFrame->mvbOutlier[i] = false;

          kpUn = pFrame->mvKeysRight[i - Nleft];
          Eigen::Matrix<double, 2, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y;

          auto* e = new EdgeMonoOnlyPose(pMP->GetWorldPos(), 1);

          e->setVertex(0, VP);
          e->setMeasurement(obs);

          // Add here uncerteinty
          const float unc2 = pFrame->mpCamera->uncertainty2(obs);

          const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          auto* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberMono);

          optimizer.addEdge(e);

          vpEdgesMono.push_back(e);
          vnIndexEdgeMono.push_back(i);
        }
      }
    }
  }

  nInitialCorrespondences =
      nInitialMonoCorrespondences + nInitialStereoCorrespondences;

  // Set Previous Frame Vertex
  Frame* pFp = pFrame->mpPrevFrame;

  auto* VPk = new VertexPose(pFp);
  VPk->setId(4);
  VPk->setFixed(false);
  optimizer.addVertex(VPk);
  auto* VVk = new VertexVelocity(pFp);
  VVk->setId(5);
  VVk->setFixed(false);
  optimizer.addVertex(VVk);
  auto* VGk = new VertexGyroBias(pFp);
  VGk->setId(6);
  VGk->setFixed(false);
  optimizer.addVertex(VGk);
  auto* VAk = new VertexAccBias(pFp);
  VAk->setId(7);
  VAk->setFixed(false);
  optimizer.addVertex(VAk);

  auto* ei = new EdgeInertial(pFrame->mpImuPreintegratedFrame);

  ei->setVertex(0, VPk);
  ei->setVertex(1, VVk);
  ei->setVertex(2, VGk);
  ei->setVertex(3, VAk);
  ei->setVertex(4, VP);
  ei->setVertex(5, VV);
  optimizer.addEdge(ei);

  auto* egr = new EdgeGyroRW();
  egr->setVertex(0, VGk);
  egr->setVertex(1, VG);
  Eigen::Matrix3d InfoG =
      pFrame->mpImuPreintegrated->C.block<3, 3>(9, 9).cast<double>().inverse();
  egr->setInformation(InfoG);
  optimizer.addEdge(egr);

  auto* ear = new EdgeAccRW();
  ear->setVertex(0, VAk);
  ear->setVertex(1, VA);
  Eigen::Matrix3d InfoA = pFrame->mpImuPreintegrated->C.block<3, 3>(12, 12)
                              .cast<double>()
                              .inverse();
  ear->setInformation(InfoA);
  optimizer.addEdge(ear);

  if (pFrame->mpOdomPreintegratedFrame) {
    auto* eo = new EdgeWOdometry(pFrame->mpOdomPreintegratedFrame);
    eo->setVertex(0, VPk);
    eo->setVertex(1, VP);
    optimizer.addEdge(eo);
  }

  if (pPlanarConstraint) {
    auto* VPlanar = new VertexPlanar();
    VPlanar->setEstimate(pPlanarConstraint);
    VPlanar->setFixed(true);
    VPlanar->setId(8);
    optimizer.addVertex(VPlanar);

    auto* eplanar = new EdgePlane();
    eplanar->Tbo = pFrame->mpOdomPreintegratedFrame->Tbo.cast<double>();
    eplanar->setVertex(0, VP);
    eplanar->setVertex(1, VPlanar);
    optimizer.addEdge(eplanar);
  }

  if (!pFp->mpcpi)
    Verbose::PrintMess(
        "pFp->mpcpi does not exist!!!\nPrevious Frame " + to_string(pFp->mnId),
        Verbose::VERBOSITY_NORMAL);

  auto* ep = new EdgePriorPoseImu(pFp->mpcpi);

  ep->setVertex(0, VPk);
  ep->setVertex(1, VVk);
  ep->setVertex(2, VGk);
  ep->setVertex(3, VAk);
  auto* rkp = new g2o::RobustKernelHuber;
  ep->setRobustKernel(rkp);
  rkp->setDelta(5);
  optimizer.addEdge(ep);

  // We perform 4 optimizations, after each optimization we classify observation
  // as inlier/outlier At the next optimization, outliers are not included, but
  // at the end they can be classified as inliers again.
  const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
  const float chi2Stereo[4] = {15.6f, 9.8f, 7.815f, 7.815f};
  const int its[4] = {10, 10, 10, 10};

  int nBad = 0;
  int nBadMono = 0;
  int nBadStereo = 0;
  int nInliersMono = 0;
  int nInliersStereo = 0;
  int nInliers = 0;
  for (size_t it = 0; it < 4; it++) {
    optimizer.initializeOptimization(0);
    optimizer.optimize(its[it]);

    nBad = 0;
    nBadMono = 0;
    nBadStereo = 0;
    nInliers = 0;
    nInliersMono = 0;
    nInliersStereo = 0;
    float chi2close = 1.5f * chi2Mono[it];

    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
      EdgeMonoOnlyPose* e = vpEdgesMono[i];

      const size_t idx = vnIndexEdgeMono[i];
      bool bClose = pFrame->mvpMapPoints[idx]->mTrackDepth < 10.f;

      if (pFrame->mvbOutlier[idx]) {
        e->computeError();
      }

      const double chi2 = e->chi2();

      if ((chi2 > chi2Mono[it] && !bClose) || (bClose && chi2 > chi2close) ||
          !e->isDepthPositive()) {
        pFrame->mvbOutlier[idx] = true;
        e->setLevel(1);
        nBadMono++;
      } else {
        pFrame->mvbOutlier[idx] = false;
        e->setLevel(0);
        nInliersMono++;
      }

      if (it == 2) e->setRobustKernel(nullptr);
    }

    for (size_t i = 0; i < vpEdgesStereo.size(); ++i) {
      EdgeStereoOnlyPose* e = vpEdgesStereo[i];

      const size_t idx = vnIndexEdgeStereo[i];

      if (pFrame->mvbOutlier[idx]) {
        e->computeError();
      }

      const double chi2 = e->chi2();

      if (chi2 > chi2Stereo[it]) {
        pFrame->mvbOutlier[idx] = true;
        e->setLevel(1);
        nBadStereo++;
      } else {
        pFrame->mvbOutlier[idx] = false;
        e->setLevel(0);
        nInliersStereo++;
      }

      if (it == 2) e->setRobustKernel(nullptr);
    }

    nInliers = nInliersMono + nInliersStereo;
    nBad = nBadMono + nBadStereo;

    if (optimizer.edges().size() < 10) {
      break;
    }
  }

  // Recover if not enough inliers
  if ((nInliers < 30) && !bRecInit) {
    nBad = 0;
    const float chi2MonoOut = 18.f;
    const float chi2StereoOut = 24.f;
    EdgeMonoOnlyPose* e1;
    EdgeStereoOnlyPose* e2;
    for (size_t i = 0, iend = vnIndexEdgeMono.size(); i < iend; i++) {
      const size_t idx = vnIndexEdgeMono[i];
      e1 = vpEdgesMono[i];
      e1->computeError();
      if (e1->chi2() < chi2MonoOut)
        pFrame->mvbOutlier[idx] = false;
      else
        nBad++;
    }
    for (size_t i = 0, iend = vnIndexEdgeStereo.size(); i < iend; i++) {
      const size_t idx = vnIndexEdgeStereo[i];
      e2 = vpEdgesStereo[i];
      e2->computeError();
      if (e2->chi2() < chi2StereoOut)
        pFrame->mvbOutlier[idx] = false;
      else
        nBad++;
    }
  }

  nInliers = nInliersMono + nInliersStereo;

  // Recover optimized pose, velocity and biases
  pFrame->SetImuPoseVelocity(VP->estimate().Rwb.cast<float>(),
                             VP->estimate().twb.cast<float>(),
                             VV->estimate().cast<float>());
  Vector6d b;
  b << VG->estimate(), VA->estimate();
  pFrame->mImuBias = IMU::Bias((float)b[3], (float)b[4], (float)b[5],
                               (float)b[0], (float)b[1], (float)b[2]);

  // Recover Hessian, marginalize previous frame states and generate new prior
  // for frame
  Eigen::Matrix<double, 30, 30> H;
  H.setZero();

  H.block<24, 24>(0, 0) += ei->GetHessian();

  Eigen::Matrix<double, 6, 6> Hgr = egr->GetHessian();
  H.block<3, 3>(9, 9) += Hgr.block<3, 3>(0, 0);
  H.block<3, 3>(9, 24) += Hgr.block<3, 3>(0, 3);
  H.block<3, 3>(24, 9) += Hgr.block<3, 3>(3, 0);
  H.block<3, 3>(24, 24) += Hgr.block<3, 3>(3, 3);

  Eigen::Matrix<double, 6, 6> Har = ear->GetHessian();
  H.block<3, 3>(12, 12) += Har.block<3, 3>(0, 0);
  H.block<3, 3>(12, 27) += Har.block<3, 3>(0, 3);
  H.block<3, 3>(27, 12) += Har.block<3, 3>(3, 0);
  H.block<3, 3>(27, 27) += Har.block<3, 3>(3, 3);

  H.block<15, 15>(0, 0) += ep->GetHessian();

  int tot_in = 0, tot_out = 0;

  for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
    EdgeMonoOnlyPose* e = vpEdgesMono[i];

    const size_t idx = vnIndexEdgeMono[i];

    if (!pFrame->mvbOutlier[idx]) {
      H.block<6, 6>(15, 15) += e->GetHessian();
      tot_in++;
    } else
      tot_out++;
  }

  for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
    EdgeStereoOnlyPose* e = vpEdgesStereo[i];

    const size_t idx = vnIndexEdgeStereo[i];

    if (!pFrame->mvbOutlier[idx]) {
      H.block<6, 6>(15, 15) += e->GetHessian();
      tot_in++;
    } else
      tot_out++;
  }

  H = Marginalize(H, 0, 14);

  pFrame->mpcpi = new ConstraintPoseImu(
      VP->estimate().Rwb, VP->estimate().twb, VV->estimate(), VG->estimate(),
      VA->estimate(), H.block<15, 15>(15, 15));
  delete pFp->mpcpi;
  pFp->mpcpi = nullptr;

  return nInitialCorrespondences - nBad;
}
Eigen::MatrixXd ORB_SLAM3::WOdometryOptimization::Marginalize(
    const Eigen::MatrixXd& H, const int& start, const int& end) {
  // Goal
  // a  | ab | ac       a*  | 0 | ac*
  // ba | b  | bc  -->  0   | 0 | 0
  // ca | cb | c        ca* | 0 | c*

  // Size of block before block to marginalize
  const int a = start;
  // Size of block to marginalize
  const int b = end - start + 1;
  // Size of block after block to marginalize
  const long c = H.cols() - (end + 1);

  // Reorder as follows:
  // a  | ab | ac       a  | ac | ab
  // ba | b  | bc  -->  ca | c  | cb
  // ca | cb | c        ba | bc | b

  Eigen::MatrixXd Hn = Eigen::MatrixXd::Zero(H.rows(), H.cols());
  if (a > 0) {
    Hn.block(0, 0, a, a) = H.block(0, 0, a, a);
    Hn.block(0, a + c, a, b) = H.block(0, a, a, b);
    Hn.block(a + c, 0, b, a) = H.block(a, 0, b, a);
  }
  if (a > 0 && c > 0) {
    Hn.block(0, a, a, c) = H.block(0, a + b, a, c);
    Hn.block(a, 0, c, a) = H.block(a + b, 0, c, a);
  }
  if (c > 0) {
    Hn.block(a, a, c, c) = H.block(a + b, a + b, c, c);
    Hn.block(a, a + c, c, b) = H.block(a + b, a, c, b);
    Hn.block(a + c, a, b, c) = H.block(a, a + b, b, c);
  }
  Hn.block(a + c, a + c, b, b) = H.block(a, a, b, b);

  // Perform marginalization (Schur complement)
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      Hn.block(a + c, a + c, b, b), Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType singularValues_inv =
      svd.singularValues();
  for (int i = 0; i < b; ++i) {
    if (singularValues_inv(i) > 1e-6)
      singularValues_inv(i) = 1.0 / singularValues_inv(i);
    else
      singularValues_inv(i) = 0;
  }
  Eigen::MatrixXd invHb = svd.matrixV() * singularValues_inv.asDiagonal() *
                          svd.matrixU().transpose();
  Hn.block(0, 0, a + c, a + c) =
      Hn.block(0, 0, a + c, a + c) -
      Hn.block(0, a + c, a + c, b) * invHb * Hn.block(a + c, 0, b, a + c);
  Hn.block(a + c, a + c, b, b) = Eigen::MatrixXd::Zero(b, b);
  Hn.block(0, a + c, a + c, b) = Eigen::MatrixXd::Zero(a + c, b);
  Hn.block(a + c, 0, b, a + c) = Eigen::MatrixXd::Zero(b, a + c);

  // Inverse reorder
  // a*  | ac* | 0       a*  | 0 | ac*
  // ca* | c*  | 0  -->  0   | 0 | 0
  // 0   | 0   | 0       ca* | 0 | c*
  Eigen::MatrixXd res = Eigen::MatrixXd::Zero(H.rows(), H.cols());
  if (a > 0) {
    res.block(0, 0, a, a) = Hn.block(0, 0, a, a);
    res.block(0, a, a, b) = Hn.block(0, a + c, a, b);
    res.block(a, 0, b, a) = Hn.block(a + c, 0, b, a);
  }
  if (a > 0 && c > 0) {
    res.block(0, a + b, a, c) = Hn.block(0, a, a, c);
    res.block(a + b, 0, c, a) = Hn.block(a, 0, c, a);
  }
  if (c > 0) {
    res.block(a + b, a + b, c, c) = Hn.block(a, a, c, c);
    res.block(a + b, a, c, b) = Hn.block(a, a + c, c, b);
    res.block(a, a + b, b, c) = Hn.block(a + c, a, b, c);
  }

  res.block(a, a, b, b) = Hn.block(a + c, a + c, b, b);

  return res;
}

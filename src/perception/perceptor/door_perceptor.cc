//
// Created by alex on 30.09.21.
//

#include <diploma_2021/logic/perception/perceptor/door_perceptor.hh>
#include <bcr_core/model/map/joint/joint_state.hh>

#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <bcr_core/model/map/link/door_link.hh>
#include <somatic_math/EulerAngles.hh>

namespace diploma_2021::logic::perception {
DoorPerceptor::DoorPerceptor(boost::uuids::uuid linkUuid, const diploma_2021::logic::world::WorldHandler *world, const diploma_2021::logic::localization::sensor_processing::LidarProcessorConfig &cfg) :
    Perceptor(linkUuid, world) {

    // TODO: Offset for door?
    auto info = dynamic_cast<diploma_2021::model::DoorLink *>(world_->Proxy().Map().lock()->SearchLink(boost::lexical_cast<std::string>(uuid_)))->GetDoorData();
    diploma_2021::model::map::MapState state = diploma_2021::model::map::MapState::CreateDefaultState(*world_->Proxy().Map().lock()->Floor(0));

    auto dl = dynamic_cast<diploma_2021::model::DoorLink *>(world_->Proxy().Map().lock()->SearchLink(boost::lexical_cast<std::string>(uuid_)));
    for (float angle = info.MinAngle; angle <= info.MaxAngle; angle += inc) {
        state.UpdateJointStateValues(boost::lexical_cast<boost::uuids::uuid>(info.BoxJointId), {angle});
        auto glob = dl->GlobalTransform(state);

        auto calcDoorPoint = [&] (const double &dx, const double &dy) -> somatic::math::Vector3 { // dx -> width, dy -> thickness
            return (glob * somatic::math::Affine3From2D(0, 0, angle) * somatic::math::Affine3From2D(dx, dy, 0)).translation();
        };

        somatic::math::Vector3
            close_left = calcDoorPoint(0., info.Thickness/2.),
            close_right = calcDoorPoint(0., -info.Thickness/2.),
            far_left = calcDoorPoint(info.Width, info.Thickness/2.),
            far_right = calcDoorPoint(info.Width, -info.Thickness/2.);

        mapCut_[angle].first.push_back(close_left);
        mapCut_[angle].first.push_back(far_left);
        mapCut_[angle].first.push_back(far_right);
        mapCut_[angle].first.push_back(close_right);

        mapCut_[angle].second.emplace_back(close_left, close_right);
        mapCut_[angle].second.emplace_back(close_left, far_left);
        mapCut_[angle].second.emplace_back(far_left, far_right);
        mapCut_[angle].second.emplace_back(close_right, far_right);
    }

    lidars_.fl = {cfg.FrontLeftLidarX(), cfg.FrontLeftLidarY(), 0.};
    lidars_.fr = {cfg.FrontRightLidarX(), cfg.FrontRightLidarY(), 0.};
    lidars_.br = {cfg.BackRightLidarX(), cfg.BackRightLidarY(), 0.};
    lidars_.bl = {cfg.BackLeftLidarX(), cfg.BackLeftLidarY(), 0.};
}

void DoorPerceptor::Percept(diploma_2021::model::map::MapState &state) {
    auto beforeExecute = std::chrono::high_resolution_clock::now();
    auto dl = dynamic_cast<diploma_2021::model::DoorLink *>(world_->Proxy().Map().lock()->SearchLink(boost::lexical_cast<std::string>(uuid_)));

    /// Get points inside reach area of door
    double ts;
    auto robotState = world_->Proxy().RobotState(ts).Pose();

    auto ms = world_->Proxy().MapState(ts);
    auto glob = dl->GlobalTransform(ms);

    auto d = (robotState.translation() - glob.translation()).norm();
    if (d > recognitionDistance_) {
        //RCLCPP_DEBUG(rclcpp::get_logger("DoorPerceptor"), "D=[%lf], skipping", d);
        return;
    }

    auto points = world_->Proxy().LidarPoints(ts);
    auto info = dl->GetDoorData();

    const somatic::math::Vector3 from = glob.translation();
    auto from_2d = Eigen::Vector2f(from.x(), from.y());

    somatic::math::Vector3 min_angle = (glob * somatic::math::Affine3From2D(0, 0, info.MinAngle) * somatic::math::Affine3From2D(dl->GetDoorData().Width, 0, 0)).translation();
    min_angle -= from;
    auto min_angle_2d = Eigen::Vector2f(min_angle.x(), min_angle.y());

    somatic::math::Vector3 max_angle = (glob * somatic::math::Affine3From2D(0, 0, info.MaxAngle) *somatic::math::Affine3From2D(dl->GetDoorData().Width, 0, 0)).translation();
    max_angle -= from;
    auto max_angle_2d = Eigen::Vector2f(max_angle.x(), max_angle.y());


    auto areClockwise = [] (Eigen::Vector2f &v1, Eigen::Vector2f &v2) -> bool {
        return -v1.x()*v2.y() + v1.y()*v2.x() > 0;
    };

    /// Get points inside interest area
    inside_.clear();
    for (auto &p : points) {
        auto p3d = robotState * somatic::math::Affine3From2D(p.x(), p.y(), 0);
        p.x() = p3d.translation().x();
        p.y() = p3d.translation().y();
        auto pLocal = p;
        pLocal -= from_2d;
        if (pLocal.norm() < info.Width && !areClockwise(min_angle_2d, pLocal) && areClockwise(max_angle_2d, pLocal)) {
            inside_.push_back(p);
        }
    }

    /// Transform 2d -> 3d
    std::vector<somatic::math::Vector3> lidarPointsInsideCheckedSector;
    std::transform(inside_.begin(), inside_.end(), std::back_inserter(lidarPointsInsideCheckedSector), [] (auto &p) -> somatic::math::Vector3 {
        return {p.x(), p.y(), 0.};
    });

    Lidars globalLidars = lidars_;
    globalLidars.fl = robotState * globalLidars.fl;
    globalLidars.fr = robotState * globalLidars.fr;
    globalLidars.bl = robotState * globalLidars.bl;
    globalLidars.br = robotState * globalLidars.br;
    
    std::map<float, double> dist;
    for (const auto & cutWithAngle : mapCut_) {
        auto &angle = cutWithAngle.first;
        auto &visibleLines = visibleLines_[angle];
        visibleLines.clear();

        auto &doorPoints = cutWithAngle.second.first;
        auto &doorLines = cutWithAngle.second.second;

        std::vector<bool> vis(4, true);
        /// Check visible lines
        for (int i = 0; i < 4; i++) {
            auto &point = doorPoints[i];
            std::vector<somatic::math::LineSegment> lidarLines = {{point, globalLidars.fl}, {point, globalLidars.fr}, {point, globalLidars.br}, {point, globalLidars.bl}};
            for (int il : {(i+2)%4, (i+3)%4}) {
                for (auto &lidar : lidarLines) {
                    if (lidar.Intersects2D(doorLines[il])) {
                        vis[(i+1)%4] = false;
                        vis[i] = false;
                        break;
                    }
                }
            }
        }
        for (int i = 0; i < 4; i++) {
            if (vis[i])
                visibleLines.push_back(doorLines[i]);
        }

        /// Calculate dist to visible lines
        for (const auto &p : lidarPointsInsideCheckedSector) {
            double bl = FLT_MAX;
            for(const auto &ls : visibleLines) {
                bl = std::min(bl, ls.DistanceToSquared(p));
            }
            dist[angle] += bl;
        }
    }
    float predictedAngle = std::min_element(dist.begin(), dist.end(), [](auto &p1, auto &p2) { return p1.second < p2.second; })->first;

    state.UpdateJointStateValues(boost::lexical_cast<boost::uuids::uuid>(info.BoxJointId), {predictedAngle});
    auto afterExecute = std::chrono::high_resolution_clock::now();
    auto cnt = std::chrono::duration<double>(afterExecute - beforeExecute).count();

    //RCLCPP_WARN(rclcpp::get_logger("DoorPerceptor"), "Predicted Angle = [%f]; Time=[%lf] seconds", predictedAngle, cnt);
}

void DoorPerceptor::Draw(bcr::transport::visualization::Graphics &g) {
    auto dl = dynamic_cast<diploma_2021::model::DoorLink *>(world_->Proxy().Map().lock()->SearchLink(boost::lexical_cast<std::string>(uuid_)));
    auto info = dl->GetDoorData();

    double ts;
    auto ms = world_->Proxy().MapState(ts);
    auto glob = dl->GlobalTransform(ms);
    const somatic::math::Vector3 from = glob.translation();

    somatic::math::Vector3 min_angle = (glob * somatic::math::Affine3From2D(0, 0, info.MinAngle) *somatic::math::Affine3From2D(dl->GetDoorData().Width, 0, 0)).translation();
    g.DrawLine(somatic::math::LineSegment(from, min_angle), bcr::transport::visualization::color::BLUE);

    somatic::math::Vector3 max_angle = (glob * somatic::math::Affine3From2D(0, 0, info.MaxAngle) *  somatic::math::Affine3From2D(dl->GetDoorData().Width, 0, 0)).translation();
    g.DrawLine(somatic::math::LineSegment(from, max_angle), bcr::transport::visualization::color::BLUE);

    double pred_angle = ms.GetJointState(boost::lexical_cast<boost::uuids::uuid>(info.BoxJointId))->Values().front();
    somatic::math::Vector3 current_angle = (glob * somatic::math::Affine3From2D(0, 0, pred_angle) *  somatic::math::Affine3From2D(dl->GetDoorData().Width, 0, 0)).translation();
    //g.DrawLine(somatic::math::LineSegment(from, current_angle), bcr::transport::visualization::color::BLUE);

    g.DrawArc(somatic::math::Arc::Create(from, min_angle, max_angle, 1), bcr::transport::visualization::color::BLUE);

    std::vector<somatic::math::Vector3> points_inside;
    std::transform(inside_.begin(), inside_.end(), std::back_inserter(points_inside), [](const auto &p) -> somatic::math::Vector3 {
        return {p.x(), p.y(), 0.01};
    });
    //g.DrawPoints(points_inside, bcr::transport::visualization::color::AQUA);
    g.DrawPoints(points_inside, bcr::transport::visualization::color::GREEN);

/*    g.DrawLines(visibleLines_[pred_angle], bcr::transport::visualization::color::AQUA);

    Lidars globalLidars = lidars_;
    globalLidars.fl = world_->Proxy().RobotState(ts).Pose() * globalLidars.fl;
    globalLidars.fr = world_->Proxy().RobotState(ts).Pose() * globalLidars.fr;
    globalLidars.bl = world_->Proxy().RobotState(ts).Pose() * globalLidars.bl;
    globalLidars.br = world_->Proxy().RobotState(ts).Pose() * globalLidars.br;
    g.DrawPoints({globalLidars.fl, globalLidars.fr, globalLidars.bl, globalLidars.br}, bcr::transport::visualization::color::RED);


    for (int i = 0; i < 4; i++) {
        auto &point = mapCut_[pred_angle].first[i];
        std::vector<somatic::math::LineSegment> lidarLines = {
                {point, globalLidars.fl},
                {point, globalLidars.fr},
                {point, globalLidars.br},
                {point, globalLidars.bl}
        };
        g.DrawLines(lidarLines, bcr::transport::visualization::color::OLIVE);
    }*/
}
}
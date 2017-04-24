#pragma once

#include "srrg_core_viewers/simple_viewer.h"
#include "srrg_gl_helpers/opengl_primitives.h"
#include "srrg_core_map/map_node_list.h"
#include "srrg_core_map/binary_node_relation.h"

namespace srrg_map_navigator {


class NavigatorViewer: public srrg_core_viewers::SimpleViewer {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    srrg_core_map::MapNodeList nodes;
    srrg_core_map::BinaryNodeRelationSet relations;
    virtual void draw();
    virtual void init();
    void fill();
    void selectLocalMap(srrg_core_map::MapNode* n, srrg_core_map::MapNodeSet &neighbors);
    inline void setRobotPose(const Eigen::Isometry3f& robot_pose){_robot_pose = robot_pose;}
protected:
    std::map<int, srrg_core_map::MapNode*> _names_map;
    srrg_core_map::MapNode* _current_object;
    std::set<srrg_core_map::MapNode*> _current_neighbors;
    Eigen::Isometry3f _robot_pose;
};

}

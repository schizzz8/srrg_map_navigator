#include <fstream>
#include "srrg_system_utils/system_utils.h"
#include "srrg_core_map/cloud.h"
#include "srrg_core_map/image_map_node.h"
#include "srrg_core_map/local_map_with_traversability.h"
#include "srrg_map_navigator_viewers/map_navigator_viewer.h"
#include "qapplication.h"
#include <stdexcept>
#include "srrg_boss/deserializer.h"
#include "srrg_boss/trusted_loaders.h"
#include "srrg_core_map/pinhole_camera_info.h"

#define rad2deg(angleRadians) (angleRadians * 180.0 / M_PI)
using namespace std;
using namespace srrg_boss;
using namespace srrg_core;
using namespace srrg_core_map;
using namespace srrg_map_navigator;

BaseCameraInfo cinfo;
ImageMapNode tnode;
LocalMap lmap;
BinaryNodeRelation rel;

typedef std::map<MapNode*,MapNodeSet> MapNodePtrMapNodeSetMap;

const char* banner[]= {
    "fps_map_navigator_app: example on how to show load a set of boss objects constituting a boss map",
    "usage:",
    " fps_map_navigator_app  <boss filename>",
    0
};

int main (int argc, char** argv) {
    if (argc<2 || ! strcmp(argv[1],"-h")) {
        printBanner(banner);
        return 0 ;
    }

    bool first = true;
    Eigen::Isometry3f robot_pose = Eigen::Isometry3f::Identity();
    Eigen::Isometry3f current_transform = Eigen::Isometry3f::Identity();
    Eigen::Isometry3f offset = Eigen::Isometry3f::Identity();
    Eigen::Isometry3f camera_pose = Eigen::Isometry3f::Identity();
    Eigen::Isometry3f camera_transform = Eigen::Isometry3f::Identity();

    float v_step = 0.1;
    float w_step = 0.1;

    std::list<Serializable*> objects;
    Deserializer des;
    des.setFilePath(argv[1]);
    Serializable* o;

    MapNodeList nodes;
    BinaryNodeRelationSet edges;
    LocalMapWithTraversability* current_map = new LocalMapWithTraversability;
    MapNodeSet current_neighbors;
    MapNodePtrMapNodeSetMap neighbors_map;

    //Parsing input file
    while ( (o = des.readObject()) ){
        LocalMapWithTraversability* lmap = dynamic_cast<LocalMapWithTraversability*>(o);
        if (lmap){
            nodes.addElement(lmap);
            if(first){
                current_map = lmap;
                current_transform = current_map->transform();
                first = false;
            }
        }
        BinaryNodeRelation* rel = dynamic_cast<BinaryNodeRelation*>(o);
        if(rel){
            LocalMapWithTraversability* from = dynamic_cast<LocalMapWithTraversability*>(rel->from());
            LocalMapWithTraversability* to = dynamic_cast<LocalMapWithTraversability*>(rel->to());
            if(from && to)
                edges.insert(std::tr1::shared_ptr<BinaryNodeRelation>(rel));
        }
        PinholeCameraInfo* info = dynamic_cast<PinholeCameraInfo*>(o);
        if(info)
            offset = info->offset();
        objects.push_back(o);
    }

    //Building neighbors map
    for(MapNodeList::iterator it = nodes.begin(); it != nodes.end(); it++)
        for(MapNodeList::iterator jt = nodes.begin(); jt != nodes.end(); jt++)
            if(it->get()->getId() != jt->get()->getId()){
                int id1 = it->get()->getId(), id2 = jt->get()->getId();
                for(BinaryNodeRelationSet::iterator kt = edges.begin(); kt != edges.end(); kt++)
                    if((*kt)->from()->getId() == id1 && (*kt)->to()->getId() == id2 ||
                            (*kt)->from()->getId() == id2 && (*kt)->to()->getId() == id1 ){
                        MapNodePtrMapNodeSetMap::iterator lt = neighbors_map.find(it->get());
                        if(lt != neighbors_map.end())
                            lt->second.insert(jt->get());
                        else{
                            MapNodeSet set;
                            set.insert(jt->get());
                            neighbors_map.insert(std::pair<MapNode*,MapNodeSet> (it->get(),set));
                        }
                    }
            }
    current_neighbors = neighbors_map[current_map];

    //Starting the viewer
    QApplication app(argc, argv);
    NavigatorViewer viewer;

    viewer.nodes = nodes;
    viewer.relations = edges;
    viewer.fill();
    viewer.show();

    cerr << "Read: " << objects.size() << " elements" << endl;
    cerr << "Read: " << viewer.nodes.size() << " local maps" << endl;
    cerr << "Read: " << viewer.relations.size() << " binary node relations" << endl;

    int previous_id = current_map->getId();
    cerr << "Current local map: " << previous_id << endl;

    while(1) {

        //Read command and determine robot next pose
        Eigen::Isometry3f next_pose = robot_pose;
        QKeyEvent* event=viewer.lastKeyEvent();
        if(event){
            switch(event->key()){
            case Qt::Key_Up:
                next_pose.translate(Eigen::Vector3f(v_step,0.0f,0.0f));
                break;
            case Qt::Key_Down:
                next_pose.translate(Eigen::Vector3f(-v_step,0.0f,0.0f));
                break;
            case Qt::Key_Right:
                next_pose.rotate(Eigen::AngleAxisf(-w_step,Eigen::Vector3f(0,0,1)));
                break;
            case Qt::Key_Left:
                next_pose.rotate(Eigen::AngleAxisf(w_step,Eigen::Vector3f(0,0,1)));
                break;
            case Qt::Key_1:
                camera_transform = Eigen::Isometry3f::Identity();
                break;
            case Qt::Key_2:
                camera_transform = Eigen::AngleAxisf(M_PI/5.0f,Eigen::Vector3f(-1,0,0));
                camera_transform.translate(Eigen::Vector3f(0,0,-2));
                break;
            case Qt::Key_3:
                camera_transform = Eigen::AngleAxisf(M_PI_2,Eigen::Vector3f(-1,0,0));
                camera_transform.translate(Eigen::Vector3f(0,0,-4));
                break;
            case Qt::Key_Escape:
                return 0;
            default:;
            }
            viewer.keyEventProcessed();
        }

        if(previous_id != current_map->getId()){
            previous_id = current_map->getId();
            cerr << "Current local map: " << previous_id << endl;
        }


        //Check traversability
        Eigen::Vector3f new_position = next_pose.translation();
        //cerr << "Next pose: " << new_position.transpose() << endl;
        if(current_map->isTraversable(new_position)){
            robot_pose = next_pose;
            //cerr << "Next pose is traversable!" << endl;
        } else {
            //cerr << "Next pose is not traversable!" << endl;
            for(MapNodeSet::iterator it = current_neighbors.begin(); it != current_neighbors.end(); it++){
                LocalMapWithTraversability* neighbor = dynamic_cast<LocalMapWithTraversability*>(*it);
                Eigen::Vector3f transformed_position = neighbor->transform().inverse()*current_transform*new_position;
                if(neighbor->isTraversable(transformed_position)){
                    robot_pose = neighbor->transform().inverse()*current_transform*next_pose;
                    current_map = neighbor;
                    current_transform = current_map->transform();
                    current_neighbors = neighbors_map[current_map];
                    //cerr << "Switching to " << current_map->getId() << endl;
                    break;
                }
            }
        }
        viewer.selectLocalMap(current_map,current_neighbors);
        viewer.setRobotPose(current_transform*robot_pose);

        //Update camera pose
        camera_pose = current_transform*robot_pose*offset;
        camera_pose = camera_pose*camera_transform;
        Eigen::Vector3f pos = camera_pose.translation();
        Eigen::Vector3f dir = camera_pose.rotation()*Eigen::Vector3f(0,0,1);
        viewer.camera()->setPosition(qglviewer::Vec(pos.x(),pos.y(),pos.z()));
        viewer.camera()->setUpVector(qglviewer::Vec(0.0f, 0.0f, 1.0f));
        viewer.camera()->lookAt(qglviewer::Vec(pos.x()+dir.x(),pos.y()+dir.y(),pos.z()+dir.z()));

        viewer.updateGL();
        app.processEvents();
        usleep(20000);

    }
    app.exec();
    return 1;
}

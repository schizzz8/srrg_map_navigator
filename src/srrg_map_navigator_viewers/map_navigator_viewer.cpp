#include "map_navigator_viewer.h"
#include "qevent.h"
#include <cstring>

namespace srrg_map_navigator{
using namespace std;
using namespace Eigen;
using namespace srrg_gl_helpers;
using namespace srrg_core_map;

class StandardCamera: public qglviewer::Camera {
public:
    StandardCamera(): _standard(true) {}

    float zNear() const {
        if(_standard) { return 0.001f; }
        else { return Camera::zNear(); }
    }

    float zFar() const {
        if(_standard) { return 20.0f; }
        else { return Camera::zFar(); }
    }

    bool standard() const { return _standard; }
    void setStandard(bool s) { _standard = s; }

protected:
    bool _standard;
};

void NavigatorViewer::init(){
    QGLViewer::init();
    //Set background color light yellow.
    setBackgroundColor(QColor::fromRgb(255, 255, 194));

    // Set some default settings.
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_POINT_SMOOTH);
    glEnable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_NORMALIZE);
    glShadeModel(GL_FLAT);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Don't save state.
    setStateFileName(QString::null);

    setMouseBinding(Qt::NoModifier, Qt::RightButton, NO_CLICK_ACTION);
    setMouseBinding(Qt::NoModifier, Qt::LeftButton, NO_CLICK_ACTION);
    setWheelBinding(Qt::NoModifier,CAMERA,NO_MOUSE_ACTION);

    // Replace camera.
    qglviewer::Camera *oldcam = camera();
    qglviewer::Camera *cam = new StandardCamera();
    setCamera(cam);
    cam->setPosition(qglviewer::Vec(0.0f, 0.0f, 0.0f));
    cam->setUpVector(qglviewer::Vec(0.0f, -1.0f, 0.0f));
    cam->lookAt(qglviewer::Vec(0.0f, 0.0f, 1.0f));
    delete oldcam;

}


void NavigatorViewer::draw() {

    glPushMatrix();
    glMultMatrix(_robot_pose);
    drawBox(0.1,0.1,0.1);
    glPopMatrix();

    glColor3f(0.5, 0.5, 0.8);
    _current_object->draw(ATTRIBUTE_SHOW | ATTRIBUTE_SELECTED,-1);
    for(std::set<MapNode*>::iterator it = _current_neighbors.begin(); it != _current_neighbors.end(); it++){
        MapNode* n = *it;
        glColor3f(0.8, 0.5, 0.8);
        n->draw(ATTRIBUTE_SHOW | ATTRIBUTE_SELECTED,-1);
    }
    for (BinaryNodeRelationSet::iterator it = relations.begin(); it!=relations.end(); it++) {
        if (! (*it)->parent())
            (*it)->draw();
    }
}

void NavigatorViewer::fill() {
    _names_map.clear();
    for (MapNodeList::iterator it = nodes.begin(); it!=nodes.end(); it++) {
        MapNode* n = it->get();
        int name = n->getId();
        n->draw(ATTRIBUTE_SHOW, name);
        _names_map.insert(make_pair(name, n));
    }
}

void NavigatorViewer::selectLocalMap(MapNode *n, MapNodeSet &neighbors){
    if (n->getId() < 0)
        return;
    _current_object = _names_map[n->getId()];
    for(MapNodeSet::iterator it = neighbors.begin(); it != neighbors.end(); it++){
        _current_neighbors.insert(*it);
    }
}

}

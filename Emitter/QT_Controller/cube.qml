import QtQuick
import QtQuick3D

Item {
    width: 200
    height: 200

    // Contour
    Rectangle {
        anchors.fill: parent
        color: "transparent"      // fond transparent
        border.color: "#434c5e"   // couleur du contour
        border.width: 2
        radius: 4                 // arrondi facultatif
        z: 1                      // pour que le border soit au-dessus si nécessaire
    }

    View3D {
        anchors.fill: parent
        z: 0  // en dessous du border
        environment: SceneEnvironment {
            backgroundMode: SceneEnvironment.Color

            clearColor: "#303030"
        }

        PerspectiveCamera {
            id: camera
            position: Qt.vector3d(0, 0, 400)
        }

        DirectionalLight {
            eulerRotation.x: -45
        }

        Model {
            id: cube
            source: "#Cube"
            scale: Qt.vector3d(2,0.5,2)

            property real pitch: 0
            property real yaw: 0
            property real roll: 0

            eulerRotation.x: pitch
            eulerRotation.y: yaw
            eulerRotation.z: roll

            materials: DefaultMaterial {
                diffuseColor: "#5E85B5"
            }
        }
    }

    function setOrientation(y, r, p) {
        cube.yaw = y
        cube.roll = r
        cube.pitch = p
    }
}

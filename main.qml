import QtQuick 2.15
import QtQuick.Window 2.15

Window {
    width: 640
    height: 480
    visible: true
    title: qsTr("Hello World with ROS2")

    Text {
        id: messageText
        text: "Waiting for ROS2 message..."
        anchors.centerIn: parent
        font.pointSize: 20
    }

    function updateMessage(msg) {
        messageText.text = msg
    }
}

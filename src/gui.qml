import QtQuick 2.15
import QtQuick.Controls 2.15

ApplicationWindow {
    visible: true
    width: 800
    height: 600
    title: "Nova5 Control GUI"

    Column {
        anchors.centerIn: parent
        spacing: 10

        Text {
            id: statusText
            text: "Status: Not Connected"
            font.pixelSize: 20
        }

        Button {
            text: "Send Command"
            onClicked: {
                // Sending commands could be triggered here
            }
        }
    }

    function updateStatus(statusMessage) {
        statusText.text = "Status: " + statusMessage
    }
}

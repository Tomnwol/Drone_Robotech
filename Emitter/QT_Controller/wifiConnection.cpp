#include <QProcess>
#include <QString>
#include <iostream>

#include "wifiConnection.hpp"

bool connectWifi(QString ssid)
{
    QProcess reConnectprocess;
    reConnectprocess.start("nmcli", {"connection","up", ssid});
    int code = 0;
    if(!reConnectprocess.waitForFinished(1000))  // 3000 ms = 3 s
    {
        //std::cout << "Timeout → killing process" << std::endl;
        reConnectprocess.kill();   // envoie SIGKILL
        reConnectprocess.waitForFinished();
        return 0; // 1 -> Non Connecte
    }else{
        code = reConnectprocess.exitCode();
        return !code;
    }
    return 0;

}


QString getCurrentWifi()
{
    QProcess process;
    process.start("nmcli", {"-t","-f","active,ssid","dev","wifi"});
    process.waitForFinished();

    QString output = process.readAllStandardOutput();
    QStringList lines = output.split("\n");

    for (const QString &line : lines) {
        if (line.startsWith("yes:")) {
            return line.section(":",1,1);
        }
    }
    return "Not connected";
}

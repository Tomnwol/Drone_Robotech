#include <QFile>
#include <QTextStream>
#include <QString>
#include <QDateTime>
#include "CSV.hpp"

bool isLogEnable = true;
QString fileName = QString("Log/log.csv");

// Objets persistants
static QFile file;
static QTextStream stream;
static int writeCount = 0;

void initLog()
{
    file.setFileName(fileName);

    if (file.open(QIODevice::Append | QIODevice::Text))
    {
        stream.setDevice(&file);

        QString timestamp = QDateTime::currentDateTime()
        .toString("yyyy-MM-dd HH:mm:ss.zzz");

        stream << "\n\n\n***********************\n"
        << timestamp
        << "\n***********************\n";

        stream.flush(); // important au démarrage
    }
}

void writeToCsv(quint16 val1, quint16 val2, quint16 val3, quint16 val4)
{
    if (!isLogEnable || !file.isOpen() || (val1 == 48 && val2 == 48 && val3 == 48 && val4 == 48))
        return;

    stream << val1 << ","
    << val2 << ","
    << val3 << ","
    << val4 << "\n";

    // Flush toutes les 50 lignes (ajuste selon besoin)
    if (++writeCount >= 50)
    {
        stream.flush();
        writeCount = 0;
    }
}

void closeLog()
{
    if (file.isOpen())
    {
        stream.flush();
        file.close();
    }
}

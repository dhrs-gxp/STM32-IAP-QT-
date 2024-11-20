#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QComboBox>
#include <QFileInfo>
#include <QFile>

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

signals:
    void sig_progressUpdate(int val);

private slots:
    void serialInfoGet(void);
    void serialConnect(void);
    void serialClose(void);
    void binFileSelect(void);
    int fileDownload(void);
    void appRun(void);
    void progressUpdate(int val);
    void recvMsgProcess(void);
    void recvMsgClean(void);
    void sendSerialMsg(void);

private:
    Ui::MainWindow *ui;

    QList<QSerialPortInfo> serialPortInfos;
    QSerialPort *serial;
    QFile *binFile;
    QString binFilePath;

};
#endif // MAINWINDOW_H

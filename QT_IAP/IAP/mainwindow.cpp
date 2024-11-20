#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QMessageBox>
#include <QFileDialog>
#include <QCoreApplication>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    ui->progressDownload->setTextVisible(true);
    ui->progressDownload->setRange(0, 100);
    ui->progressDownload->setValue(0);

    serial = new QSerialPort(this);
    binFile = new QFile(this);

    connect(ui->btnUpdate, &QPushButton::clicked,
            this, &MainWindow::serialInfoGet);
    connect(ui->btnOpen, &QPushButton::clicked,
            this, &MainWindow::serialConnect);
    connect(ui->btnClose, &QPushButton::clicked,
            this, &MainWindow::serialClose);
    connect(ui->btnChoose, &QToolButton::clicked,
            this, &MainWindow::binFileSelect);
    connect(ui->btnDownload, &QPushButton::clicked,
            this, &MainWindow::fileDownload);
    connect(ui->btnRun, &QPushButton::clicked,
            this, &MainWindow::appRun);

    connect(ui->btnCleanRecv, &QPushButton::clicked,
            this, &MainWindow::recvMsgClean);
    connect(ui->btnSend, &QPushButton::clicked,
            this, &MainWindow::sendSerialMsg);

    connect(this, &MainWindow::sig_progressUpdate,
            this, &MainWindow::progressUpdate);

    connect(serial, &QSerialPort::readyRead,
            this, &MainWindow::recvMsgProcess);

}

MainWindow::~MainWindow()
{
    delete binFile;
    delete serial;
    delete ui;
}

void MainWindow::serialInfoGet()
{
    serialPortInfos = QSerialPortInfo::availablePorts();
    ui->boxSerial->clear();
    foreach (const QSerialPortInfo &serialPortInfo, serialPortInfos) {
        ui->boxSerial->addItem(serialPortInfo.portName()
                               + serialPortInfo.description());
    }
}

void MainWindow::serialConnect()
{
    serial->setPortName(serialPortInfos.at(ui->boxSerial->currentIndex()).portName());
    serial->setBaudRate(QSerialPort::Baud115200);
    serial->setDataBits(QSerialPort::Data8);
    serial->setParity(QSerialPort::NoParity);
    serial->setStopBits(QSerialPort::OneStop);
    serial->setFlowControl(QSerialPort::NoFlowControl);

    if (serial->open(QIODevice::ReadWrite))
        ui->btnOpen->setEnabled(false);
    else
        QMessageBox::warning(this, tr("notice!"), "串口打开失败",
                             QMessageBox::Ok, QMessageBox::Ok);
}

void MainWindow::serialClose()
{
    serial->close();
    ui->btnOpen->setEnabled(true);
}

void MainWindow::binFileSelect()
{
    binFilePath = QFileDialog::getOpenFileName(NULL, "", "", tr("(*.bin)"));
    if(!binFilePath.isEmpty()) ui->lineFile->setText(binFilePath);
}

int MainWindow::fileDownload()
{
    ui->progressDownload->setValue(0);
    binFile = new QFile(binFilePath);
    int totalSize = binFile->size();
    if(!binFile->open(QIODevice::ReadOnly | QIODevice::Unbuffered)) {
        QMessageBox::warning(this, tr("notice!"), "文件打开失败",
                             QMessageBox::Ok, QMessageBox::Ok);
        return -1;
    }
    else if(totalSize == 0 || totalSize > 512*1024) {
        QMessageBox::warning(this, tr("notice!"), "文件大小错误",
                             QMessageBox::Ok, QMessageBox::Ok);
        return -1;
    }
    else if(!serial->isOpen()) {
        QMessageBox::warning(this, tr("notice!"), "串口未打开",
                             QMessageBox::Ok, QMessageBox::Ok);
        return -1;
    }
    else {

        int len = 0;
        int sendSize = 0;
        const int bufferSize = 1024;
        char buffer[bufferSize] = {0};

        disconnect(serial, &QSerialPort::readyRead,
                this, &MainWindow::recvMsgProcess);

        serial->write("Download", 8);
        serial->flush();
        while(serial->waitForBytesWritten(1000) == false);
        if(serial->waitForReadyRead(3000) == false) {
            QMessageBox::warning(this, tr("notice!"), "数据传输超时：Download",
                                 QMessageBox::Ok, QMessageBox::Ok);
            connect(serial, &QSerialPort::readyRead, this, &MainWindow::recvMsgProcess);
            return -1;
        }
        if(serial->readAll().compare("Download") != 0) {
            QMessageBox::warning(this, tr("notice!"), "Download回复错误",
                                 QMessageBox::Ok, QMessageBox::Ok);
            connect(serial, &QSerialPort::readyRead, this, &MainWindow::recvMsgProcess);
            return -1;
        }

        serial->write(QString::number(totalSize).toStdString().c_str(),
                      QString::number(totalSize).toStdString().size());
        serial->flush();
        if(serial->waitForReadyRead(2000) == false) {
            QMessageBox::warning(this, tr("notice!"), "数据传输超时：fileSize",
                                 QMessageBox::Ok, QMessageBox::Ok);
            connect(serial, &QSerialPort::readyRead, this, &MainWindow::recvMsgProcess);
            return -1;
        }
        if(serial->readAll().toInt() != totalSize) {
            QMessageBox::warning(this, tr("notice!"), "fileSize回复错误",
                                 QMessageBox::Ok, QMessageBox::Ok);
            connect(serial, &QSerialPort::readyRead, this, &MainWindow::recvMsgProcess);
            return -1;
        }

        while((len = binFile->read(buffer, bufferSize)) > 0) {
            sendSize += serial->write(buffer, len);
            serial->flush();
            if(serial->waitForReadyRead(2000) == false) {
                QMessageBox::warning(this, tr("notice!"), "数据传输超时：fileData",
                                     QMessageBox::Ok, QMessageBox::Ok);
                connect(serial, &QSerialPort::readyRead, this, &MainWindow::recvMsgProcess);
                return -1;
            }
            QByteArray temp = serial->readAll();
            if(temp.compare("OK") != 0 && temp.compare("download over") != 0) {
                QMessageBox::warning(this, tr("notice!"), "下载回复错误",
                                     QMessageBox::Ok, QMessageBox::Ok);
                connect(serial, &QSerialPort::readyRead, this, &MainWindow::recvMsgProcess);
                return -1;
            }
            QCoreApplication::processEvents();
            emit sig_progressUpdate(100*sendSize/totalSize);
        }
        connect(serial, &QSerialPort::readyRead, this, &MainWindow::recvMsgProcess);
    }
    delete binFile;
    return 0;
}

void MainWindow::progressUpdate(int val)
{
    ui->progressDownload->setValue(val);
}

void MainWindow::appRun(void)
{
    serial->write("Execute", 7);
}


void MainWindow::recvMsgProcess()
{
    ui->textRecv->append(">> " + serial->readAll());
}

void MainWindow::recvMsgClean()
{
    ui->textRecv->clear();
}

void MainWindow::sendSerialMsg()
{
    QString temp = ui->textSend->toPlainText();
    serial->write(temp.toStdString().c_str(), temp.length());
}


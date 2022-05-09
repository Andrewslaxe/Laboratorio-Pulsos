#include "widget.h"
#include "ui_widget.h"
#include <QVector>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include "stdio.h"
#include "stdlib.h"
#include "cstring"

#define Start 0x06
#define Stop 0x07
double RPMMin = NULL, RPMMax = 0, Flag = 0, RPMAct = NULL, Mediciones = 0;
double siz=0; //Mediciones por size RPMAct por RPm

Widget::Widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Widget)
{
    ui->setupUi(this);
    serialBuffer = "";
    parsed_data = "";

    foreach(const QSerialPortInfo &serialPortInfo, QSerialPortInfo::availablePorts()){
        QString pname = serialPortInfo.portName();
        ui->comboBox->addItem(pname);
    }
    Widget::PlotConfig();
}

Widget::~Widget()
{
   if(ttl->isOpen()){
       ttl->close();
       QObject::disconnect(ttl, SIGNAL(readyRead()), this, SLOT(readSerial()));
   }

    delete ui;
}
void Widget::PlotConfig(){
    ui->myplot->addGraph(ui->myplot->xAxis, ui->myplot->yAxis);
    ui->myplot->addGraph(ui->myplot->xAxis, ui->myplot->yAxis2);
    ui->myplot->graph(0)->setData(x, y);
    ui->myplot->graph(1)->setData(z, w);
    ui->myplot->graph(1)->setPen(QPen(Qt::red));
    ui->myplot->graph(0)->setName("RPM");
    ui->myplot->graph(1)->setName("Acceleracion");
    ui->myplot->plotLayout()->insertRow(0);
    ui->myplot->plotLayout()->addElement(0, 0, new QCPTextElement(ui->myplot, "Velocidad - Motor DC", QFont("sans", 12, QFont::Bold)));

    ui->myplot->legend->setVisible(true);
    QFont legendFont = font();  // start out with MainWindow's font..
    legendFont.setPointSize(8); // and make a bit smaller for legend
    ui->myplot->legend->setFont(legendFont);
    ui->myplot->legend->setBrush(QBrush(QColor(255,255,255,230)));
    // by default, the legend is in the inset layout of the main axis rect. So this is how we access it to change legend placement:
    ui->myplot->axisRect()->insetLayout()->setInsetAlignment(3, Qt::AlignTop|Qt::AlignRight);

    ui->myplot->xAxis->setLabel("Time Relative");
    ui->myplot->yAxis->setLabel("RPM.");
    ui->myplot->xAxis->setRange(0, 1000);
    ui->myplot->yAxis->setRange(0, 10000);
    ui->myplot->yAxis2->setLabel("mA");
    ui->myplot->yAxis2->setRange(0, 1000);
    ui->myplot->yAxis2->setVisible(true);

    ui->myplot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    ui->myplot->replot();
}

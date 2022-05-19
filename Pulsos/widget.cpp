#include "widget.h"
#include "ui_widget.h"
#include <QVector>
#include <QSerialPort>
#include <QSerialPortInfo>
#include "stdio.h"
#include "stdlib.h"
#include "cstring"

#define Start 0x06
#define Stop 0x07
double RPMMin=0,RPMMax=0,Flag=0,RPM=0,CurrentAct=0;
double siz=0;


Widget::Widget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Widget)
{

    ui->setupUi(this);
    ttl = new QSerialPort(this);
    serialBuffer = "";
    parsed_data = "";

    foreach(const QSerialPortInfo &serialPortInfo, QSerialPortInfo::availablePorts()){
        QString pname = serialPortInfo.portName();
        ui->comboBox->addItem(pname);
    }

    setupPlot();
}

Widget::~Widget()
{
   if(ttl->isOpen()){
       ttl->close();
       QObject::disconnect(ttl, SIGNAL(readyRead()), this, SLOT(readSerial()));
   }

    delete ui;
}

void Widget::setupPlot(){
    ui->customPlot->addGraph(ui->customPlot->xAxis, ui->customPlot->yAxis);
    //ui->customPlot->addGraph(ui->customPlot->xAxis, ui->customPlot->yAxis2);
    ui->customPlot->graph(0)->setData(x, RpmGraph);
    //ui->customPlot->graph(1)->setData(y, CurrentGraph);
    //ui->customPlot->graph(1)->setPen(QPen(Qt::red));
    ui->customPlot->graph(0)->setName("RPM");
    //ui->customPlot->graph(1)->setName("Corriente");
    ui->customPlot->plotLayout()->insertRow(0);
    ui->customPlot->plotLayout()->addElement(0, 0, new QCPTextElement(ui->customPlot, "Velocidad - Motor DC", QFont("sans", 12, QFont::Bold)));
    ui->customPlot->legend->setVisible(true);
    QFont legendFont = font();
    legendFont.setPointSize(9);
    ui->customPlot->legend->setFont(legendFont);
    ui->customPlot->legend->setBrush(QBrush(QColor(255,255,255,230)));
    ui->customPlot->xAxis->setLabel("Time Relative");
    ui->customPlot->yAxis->setLabel("RPM.");
    ui->customPlot->xAxis->setRange(0, 1000);
    ui->customPlot->yAxis->setRange(0, 10000);
    ui->customPlot->yAxis2->setLabel("mA");
    ui->customPlot->yAxis2->setRange(0, 1000);
    ui->customPlot->yAxis2->setVisible(true);
    ui->customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    ui->customPlot->replot();
}

void Widget::makeplot(){

    siz++;
    /*if(siz>254){
     * double i=0;
        for(i=0;i<253;i++){
            RpmGraph.operator[](i) = RpmGraph.operator[](i+1);

        }
        RpmGraph.operator[](253)=RPm;
    }*/
    CurrentGraph.append(CurrentAct);
    RpmGraph.append(RPM);
    x.append(siz);
    y.append(siz);
    ui->customPlot->graph(0)->setData(x, RpmGraph);
    //ui->customPlot->graph(1)->setData(y,CurrentGraph);
    ui->customPlot->replot();
    ui->customPlot->rescaleAxes();
    ui->customPlot->update();
}


void Widget::Send(uint8_t cmd,uint32_t Info){
    //Protocolo
    uint8_t Count,Parity=0,Tam;
    QByteArray data;
    data.clear();
    data.append(Start);
    if(Info<256){
        Tam = 1;
    }
    else if(Info<65536){
        Tam = 2;
    }
    else{
        Tam = 3;
    }
    data.append(Tam);
    data.append(cmd);

    if(Tam==1){
        data.append(Info);
    }
    else if(Tam==2) {
        data.append(0x00FF & (Info >> 8));
        data.append(0x00FF & Info);
    }
    else if(Tam==3){
        data.append(0x00FF & (Info >> 16 ));
        data.append(0x00FF & (Info >> 8));
        data.append(0x00FF & Info);
    }
    for(Count=0;Count<data.length();Count++){
        Parity^=data.at(Count);
    }

    data.append(Parity);
    data.append(Stop);
    ttl->write(data,data.length());
}
void Widget::readSerial()
{
    uint8_t Size,Aux,Parity=0,Check,Temp1,Temp2, Temp3;
    uint32_t Data = 0;
    QByteArray buffer ;
    serialData.clear();
    buffer.clear();
    buffer= ttl->readAll();
    serialData.append(buffer);
    ui->label_12->setText("Recibiendo datos");
    Size=serialData.at(1);
    if(serialData.at(0)==Start && serialData.at(Size+4)==Stop){

        for(Aux=3;Aux<=Size+3;Aux++){
            if(serialData.at(Aux)<0){
                serialData[Aux]=serialData.at(Aux)+256;
            }
        }
        if(Size==1){
            Data=(serialData.at(3));
        }
        else if(Size==2){
            Temp1 = (serialData.at(3));
            Temp2 = serialData.at(4);
            Data = (Temp1<< 8)| Temp2;
        }
        else if(Size==3){
            Temp1 = serialData.at(3);
            Temp2 = serialData.at(4);
            Temp3 = serialData.at(5);
            Data = (Temp1<< 16)| (Temp2<< 8)| Temp3;
        }
        for(Aux =0;Aux<Size+3;Aux++){
            Parity^=serialData.at(Aux);
        }
        Check=serialData.at(Size+3);
        if(Parity==Check){
            processSerial(Data,serialData.at(2));
        }
        else{
            qDebug()<< "Se recibió información con errores";
        }
        serialData.clear();
    }
}

void Widget::processSerial(double data,int cmd){
    uint8_t AuxSentido;
    float PosActualx = 0, PosActualy;
    QString Datos;
    Datos=QString::number(data);
    qDebug()<< Datos;
    if(cmd==1){     //Se reciben rpm
        RPM = data;
        if(Flag==0){
            RPMMin=RPM;
            Flag++;
        }
        else{
            if(RPMMin>RPM){
                RPMMin=RPM;
            }
            if(RPMMax<RPM){
                RPMMax=RPM;
            }
            ui->RpmAct->setText(QString::number(RPM));
            ui->RpmMax->setText(QString::number(RPMMax));
            ui->RpmMin->setText(QString::number(RPMMin));
        }
    }
    else if(cmd==2){    //Se recibe corriente
        CurrentAct=data;
    }
    else if(cmd == 3){ //Se recibe posición Actual en pulsos en X
        PosActualx = data / 60;
        ui->label->setText("X: "+ QString::number(PosActualx)+" mm");
    }
    else if(cmd == 4){ //Se recibe posición Actual en pulsos en Y
        PosActualy = data / 60;
        ui->label_13->setText("Y: "+QString::number(PosActualy)+" mm");
    }
    else if(cmd == 5){
        AuxSentido = data;
        switch(AuxSentido){
            case 0:
                ui->Sentidox->setText("Detenido");
            break;
            case 1:
                ui->Sentidox->setText("Horario");
            break;
            case 2:
                ui->Sentidox->setText("Antihorario");
            break;
        }
    }
    else if(cmd == 6){
        AuxSentido = data;
        switch(AuxSentido){
            case 0:
                ui->Sentidoy->setText("Detenido");
            break;
            case 1:
                ui->Sentidoy->setText("Horario");
            break;
            case 2:
                ui->Sentidoy->setText("Antihorario");
            break;
        }
    }
    makeplot();
}

void Widget::on_pushButton_2_clicked()
{
    QString ttl_port_name = ui->comboBox->currentText();
    if(ui->pushButton_2->text() == "Abrir"){
        foreach(const QSerialPortInfo &serialPortInfo, QSerialPortInfo::availablePorts()){
            if(ttl_port_name==serialPortInfo.portName()){
                ttl->setPortName(ttl_port_name);
                ttl->open(QSerialPort::ReadWrite);
                ttl->setBaudRate(QSerialPort::Baud115200);
                ttl->setDataBits(QSerialPort::Data8);
                ttl->setFlowControl(QSerialPort::NoFlowControl);
                ttl->setParity(QSerialPort::NoParity);
                ttl->setStopBits(QSerialPort::OneStop);
                QObject::connect(ttl, SIGNAL(readyRead()), this, SLOT(readSerial()));
                Send(1,5);
                ui->label_12->setText("Conectado");
                ui->pushButton_2->setText("Cerrar");

            }
            else{
                ui->label_12->setText("Puerto no disponible");
            }
        }
    }else{
        Send(0,2); //Le cierra la Flag a la stm para enviar datos
        ttl->close();
        QObject::disconnect(ttl, SIGNAL(readyRead()), this, SLOT(readSerial()));
        ui->pushButton_2->setText("Abrir");
        ui->label_12->setText("Desconectado");
    }

}

void Widget::on_pushButton_3_clicked()
{
    ui->comboBox->clear();
    foreach(const QSerialPortInfo &serialPortInfo, QSerialPortInfo::availablePorts()){
        QString pname = serialPortInfo.portName();
        ui->comboBox->addItem(pname);
    }
}


void Widget::on_pushButton_4_clicked()
{
    int PulsosDesx, PulsosDesy;
    double PosDeseadax, PosDeseaday;
    PosDeseadax = (double)ui->lineEdit_3->text().toDouble();
    PosDeseaday = (double)ui->lineEdit_4->text().toDouble();
    PulsosDesx = floor(PosDeseadax * 60);
    PulsosDesy = floor(PosDeseaday * 60);
    Send(2,PulsosDesx);
    while(!ttl->waitForBytesWritten());
    Send(3,PulsosDesy);
}



void Widget::on_pushButton_5_clicked()
{
    Send(4, 1);   //Resetea la posicion
}


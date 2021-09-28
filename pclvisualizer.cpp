#include "pclvisualizer.h"
#include "./ui_pclvisualizer.h"

PCLVisualizer::PCLVisualizer(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::PCLVisualizer)
{
    ui->setupUi(this);
}

PCLVisualizer::~PCLVisualizer()
{
    delete ui;
}


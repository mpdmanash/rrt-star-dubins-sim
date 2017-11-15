#include "renderarea.h"
#include <queue>
#include <QTimer>

RenderArea::RenderArea(QWidget *parent) : QWidget(parent)
{
    setAttribute(Qt::WA_StaticContents);
    scribbling = false;
    rrtstar = new RRTSTAR;
}

/**
 * @brief Draw the world.
 * @param painter
 */
void RenderArea::drawField(QPainter &painter)
{
    painter.save();
    painter.setRenderHint(QPainter::Antialiasing);
    QRect field;
    field.setTopLeft(QPoint(this->x(), this->y()));
    field.setBottomRight(QPoint(this->width()-1, this->height()-1));
    painter.setPen(Qt::black);
    painter.setBrush(QBrush(Qt::green));
    painter.drawRect(field);
    painter.restore();
}

/**
 * @brief Draw the start position of the bot.
 * @param painter
 */
void RenderArea::drawStartPos(QPainter &painter)
{
    painter.save();
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setPen(Qt::black);
    painter.setBrush(QBrush(Qt::red));
    painter.drawEllipse(this->x() + START_POS_X - BOT_RADIUS, this->y() + START_POS_Y - BOT_RADIUS, 2 * BOT_RADIUS, 2 * BOT_RADIUS);
    painter.restore();
}

/**
 * @brief Draw the end point.
 * @param painter
 */
void RenderArea::drawEndPos(QPainter &painter)
{
    painter.save();
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setPen(Qt::black);
    painter.setBrush(QBrush(Qt::blue));
    painter.drawEllipse(END_POS_X - BOT_RADIUS, END_POS_Y - BOT_RADIUS, 2 * BOT_RADIUS, 2 * BOT_RADIUS);
    painter.restore();
}

/**
 * @brief Draw all the rectangular obstacles.
 * @param painter
 */
void RenderArea::drawObstacles(QPainter &painter)
{
    painter.save();
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setPen(Qt::black);
    painter.setBrush(QBrush(Qt::black));
    pair<Vector2f, Vector2f> obstacle;
    for(int i = 0; i < (int)rrtstar->obstacles->obstacles.size(); i++) {
        obstacle = rrtstar->obstacles->obstacles[i];
        QPoint topLeft(obstacle.first.x() + BOT_CLEARANCE, obstacle.first.y() + BOT_CLEARANCE);
        QPoint bottomRight(obstacle.second.x() - BOT_CLEARANCE, obstacle.second.y() - BOT_CLEARANCE);
        QRect rect(topLeft, bottomRight);
        painter.drawRect(rect);
    }
    painter.restore();
}

/**
 * @brief Draw all the nodes generated in the RRTSTAR algorithm.
 * @param painter
 */
void RenderArea::drawNodes(QPainter &painter)
{
    painter.save();
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setPen(Qt::black);
    painter.setBrush(QBrush(Qt::black));
    Vector2f pos; double dubinspos1[3] = {0}; double dubinspos2[3] = {0};
    for(int i = 0; i < (int)rrtstar->nodes.size(); i++) {
        if(!BOT_FOLLOW_DUBIN){
            Vector2f parentpos = rrtstar->nodes[i]->position;
            for(int j = 0; j < (int)rrtstar->nodes[i]->children.size(); j++) {
                pos = rrtstar->nodes[i]->children[j]->position;
                painter.drawEllipse(pos.x()-1.5, pos.y()-1.5, 3, 3);
                QPointF p1(parentpos.x(), parentpos.y());
                QPointF p2(pos.x(), pos.y());
                painter.drawLine(p1, p2);
            }
        }
        pos = rrtstar->nodes[i]->position;
        painter.drawEllipse(pos.x() - NODE_RADIUS, pos.y() - NODE_RADIUS, 2 * NODE_RADIUS, 2 * NODE_RADIUS);

        if(BOT_FOLLOW_DUBIN){
            dubins_path_sample(&rrtstar->nodes[i]->path, 0, dubinspos1);
            for(int j=1; j<rrtstar->step_size; j++){
                dubins_path_sample(&rrtstar->nodes[i]->path, j, dubinspos2);
                QPointF p1(dubinspos1[0], -dubinspos1[1]);
                QPointF p2(dubinspos2[0], -dubinspos2[1]);
                painter.drawLine(p1, p2);
                dubinspos1[0] = dubinspos2[0];
                dubinspos1[1] = dubinspos2[1];
            }
        }
    }
    painter.setPen(Qt::red);
    painter.setBrush(QBrush(Qt::red));

    // if a path exists, draw it.
    for(int i = 0; i < (int)rrtstar->path.size() - 1; i++) {
        if(BOT_FOLLOW_DUBIN){
            dubins_path_sample(&rrtstar->path[i+1]->path, 0, dubinspos1);
            for(int j=1; j<rrtstar->step_size; j++){
                dubins_path_sample(&rrtstar->path[i+1]->path, j, dubinspos2);
                QPointF p1(dubinspos1[0], -dubinspos1[1]);
                QPointF p2(dubinspos2[0], -dubinspos2[1]);
                painter.drawLine(p1, p2);
                dubinspos1[0] = dubinspos2[0];
                dubinspos1[1] = dubinspos2[1];
            }
        }
        else{
            QPointF p1(rrtstar->path[i]->position.x(), rrtstar->path[i]->position.y());
            QPointF p2(rrtstar->path[i+1]->position.x(), rrtstar->path[i+1]->position.y());
            painter.drawLine(p1, p2);
        }

    }
    painter.restore();
}

void RenderArea::paintEvent(QPaintEvent *)
{
    QPainter painter(this);
    drawField(painter);
    drawStartPos(painter);
    drawEndPos(painter);
    drawObstacles(painter);
    drawNodes(painter);
    emit painting();
}

void RenderArea::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton) {
        lastMouseClickedPoint = event->pos();
        scribbling = true;
    }
}

void RenderArea::mouseMoveEvent(QMouseEvent *event)
{

}

void RenderArea::mouseReleaseEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton && scribbling) {
        QPoint curPoint = event->pos();
        rrtstar->obstacles->addObstacle(Vector2f(lastMouseClickedPoint.x(), lastMouseClickedPoint.y()), Vector2f(curPoint.x(), curPoint.y()));
        update();
        scribbling = false;
    }
}

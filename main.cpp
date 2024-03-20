#include <unordered_map>
#include <map>
#include <queue>
#include <utility>
#include <iostream>
#include <random>

//  #include <bits/stdc++.h>

using namespace std;

const int n = 200;
const int robot_num = 10;
const int berth_num = 10;
const int N = 210;

int money, boat_capacity, id;
char ch[N][N]; // 字符地图
int gds[N][N];

int berthVal[10], printBerth, printRobot = 1;//������ 

struct Square
{
    bool isBarrierExist; // 是否有障碍物
    bool isGoodExist;    // 是否有货物
    bool isRobotExist;   // 是否有机器人
    bool isBerth;        // 是否是泊位
    int goodValue;       // 货物价值
    int goodTime;        // 货物存留时间
    int generateTime;
    int goodRobotId;
    //    unordered_map<int, int> sign; // 该格子被标记的路径(经过该格子第几帧<-->机器人编号)
    Square() : goodRobotId(-1), isBarrierExist(false), isGoodExist(false), isRobotExist(false), isBerth(false), goodTime(0), goodValue(0)
    {
    }
    void generateGood(int val, int tim) // 货物生成
    {
        goodValue = val;
        isGoodExist = true;
        // goodTime = 1000;
        generateTime = tim;
    }
    // void testGood() // 货物存留计时器
    // {
    //     if (isGoodExist)
    //     {
    //         goodTime--;
    //         if (goodTime == 0)
    //         {
    //             isGoodExist = false;
    //         }
    //     }
    // }
    bool GoodExist()
    {
        return (id - generateTime <= 1000) && isGoodExist;
    }
};

Square our_map[n][n]; // 创建地图

struct Robot
{
    int x, y, goods; // 是否携带物品
    int status;      // 0表示恢复，1表示正常运行
    int mbx, mby;    // 此帧机器人所在的位置
    int aimX, aimY;  // 目标位置（added by cyh）
    int goodValue;   // 携带的货物的价值
    Robot() {}
    Robot(int startX, int startY)
    {
        x = startX;
        y = startY;
    }
} robot[robot_num + 10];

struct Berth
{
    int x;
    int y;
    int transport_time;
    int loading_speed;

    bool is_occupied; // 如果有船占用或有船驶向此泊位，则为true
    bool is_closed;
    deque<int> goods; // 待装载的货物
    Berth()
    {
        is_occupied = false;
    }

    Berth(int x, int y, int transport_time, int loading_speed)
    {
        this->x = x;
        this->y = y;
        this->transport_time = transport_time;
        this->loading_speed = loading_speed;

        is_occupied = false;
        is_closed = false;
    }
    /*
     * \brief 判断(x, y)是否在泊位内
     */
    bool isInBerth(int x, int y)
    {
        return (x >= this->x && x < this->x + 4 && y >= this->y && y < this->y + 4);
    }
} berth[berth_num + 10];
int closedBerth = 0;
int averageTransportTime;
/*
 * \brief 判断(x, y)在哪个泊位内
 */
int InWhichBerth(int x, int y)
{
    for (int i = 0; i < berth_num; i++)
    {
        if (berth[i].isInBerth(x, y))
            return i;
    }
    return -1;
}
map<pair<int, int>, int> robotMap; //(x, y), haverobot
//
// robotaction by cyh
//
int dx[4] = {0, 0, -1, 1};
int dy[4] = {1, -1, 0, 0};
int disBerth[205][205];

bool isValid(int x, int y)
{
    return (x >= 0 && x < 200 && y >= 0 && y < 200 && (!our_map[x][y].isBarrierExist));
}
void berthBfs(int x, int y)
{
    queue<pair<int, int> > q;
    q.push({x, y});
    disBerth[x][y] = 0;
    while (!q.empty())
    {
        pair<int, int> curr = q.front();
        q.pop();
        int curX = curr.first, curY = curr.second;
        for (int i = 0; i < 4; i++)
        {
            int newX = curX + dx[i];
            int newY = curY + dy[i];
            if (isValid(newX, newY) && disBerth[curX][curY] + 1 < disBerth[newX][newY])
            {
                q.push({newX, newY});
                disBerth[newX][newY] = disBerth[curX][curY] + 1;
            }
        }
    }
}
double generateRandom() {
    static random_device rd;
    static mt19937 engine(rd());
    uniform_real_distribution<double> dist(0.0, 1.0);
    return dist(engine);
}

// bool generateRandom(int x) {
//     return (rand() % 100) < x;
// }

void initDisBerth()
{
    for (int i = 0; i < 200; i++)
        for (int j = 0; j < 200; j++)
            disBerth[i][j] = 45000;
    for (int i = 0; i < berth_num; i++)
    {
        berthBfs(berth[i].x, berth[i].y);
    }
}
float valueFunctionGood(int dis, int goodX, int goodY, int value)
{
    return value / (float)(dis + disBerth[goodX][goodY]);
}


bool visitedMap[205][205];

int robotPath[45000][11];  //step, robot_id + 1
int robotStep[11][2]; // current_step, total_step;

int bfsQueue[45000][5]; // x,y,step,lastStep,action;

pair<int, int> findAimGood(int startX, int startY)
{
	float maxW = 0;
	pair<int, int> maxGoodPos;
    int front = 0, rear = 0;
    for (int i = 0; i < 200; i++)
        for (int j = 0; j < 200; j++)
            visitedMap[i][j] = false;
    visitedMap[startX][startY] = true;
    bfsQueue[rear][0] = startX;
    bfsQueue[rear][1] = startY;
    bfsQueue[rear][2] = 0;
    rear++;
    visitedMap[startX][startY] = true;
    while (front != rear)
    {
        int curX = bfsQueue[front][0], curY = bfsQueue[front][1], curDis = bfsQueue[front][2];
//        if(curDis > 200 && maxW != 0) break;
        front++;
        if (our_map[curX][curY].GoodExist() && our_map[curX][curY].goodRobotId == -1 && (!our_map[curX][curY].isBerth) && valueFunctionGood(curDis, curX, curY, our_map[curX][curY].goodValue) > maxW)
        {
            maxW = valueFunctionGood(curDis, curX, curY, our_map[curX][curY].goodValue);
            maxGoodPos = {curX, curY};
        }
        for (int i = 0; i < 4; i++)
        {
            int newX = curX + dx[i];
            int newY = curY + dy[i];
            if (isValid(newX, newY) && !visitedMap[newX][newY] && !(front <= 10 && robotMap[{newX, newY}] != 0))
            {
                visitedMap[newX][newY] = true;
                bfsQueue[rear][0] = newX;
                bfsQueue[rear][1] = newY;
                bfsQueue[rear][2] = curDis + 1;
                rear++;
            }
        }
    }
    return maxGoodPos;
}
pair<int, int> findAimBerth(int startX, int startY)
{
    int front = 0, rear = 0;
    if (our_map[startX][startY].isBerth)
        return {startX, startY};
    for (int i = 0; i < 200; i++)
        for (int j = 0; j < 200; j++)
            visitedMap[i][j] = false;
    visitedMap[startX][startY] = true;
    bfsQueue[rear][0] = startX;
    bfsQueue[rear][1] = startY;
    bfsQueue[rear][2] = 0;
    rear++;
    visitedMap[startX][startY] = true;
    while (front != rear)
    {
        int curX = bfsQueue[front][0], curY = bfsQueue[front][1], curDis = bfsQueue[front][2];
        front++;
        for (int i = 0; i < 4; i++)
        {
            int newX = curX + dx[i];
            int newY = curY + dy[i];
            if (isValid(newX, newY) && !visitedMap[newX][newY] && !(front <= 10 && robotMap[{newX, newY}] != 0))
            {
                if (our_map[newX][newY].isBerth && !berth[InWhichBerth(newX, newY)].is_closed)
                    return {newX, newY};
                visitedMap[newX][newY] = true;
                bfsQueue[rear][0] = newX;
                bfsQueue[rear][1] = newY;
                bfsQueue[rear][2] = curDis + 1;
                rear++;
            }
        }
    }
    return {startX, startY};
}
int findNextStep(int startX, int startY, int bfsId, int robotId)
{
    int lastId = bfsQueue[bfsId][3];
    robotPath[bfsQueue[bfsId][2]][robotId] = bfsQueue[bfsId][4];
    if (bfsQueue[lastId][0] == startX && bfsQueue[lastId][1] == startY)
    {
        return bfsQueue[bfsId][4];
    }
    return findNextStep(startX, startY, lastId, robotId);
}
int seq[4] = {0, 2, 1, 3};
int robotBfsToAim(int startX, int startY, int aimX, int aimY, int robotId)
{
    int front = 0, rear = 0;
    if (startX == aimX && startY == aimY)
        return -1;
    for (int i = 0; i < 200; i++)
        for (int j = 0; j < 200; j++)
            visitedMap[i][j] = false;
    bfsQueue[rear][0] = startX;
    bfsQueue[rear][1] = startY;
    bfsQueue[rear][2] = 0;
    bfsQueue[rear][3] = 0;
    rear++;
    visitedMap[startX][startY] = true;
    while (front != rear)
    {
        int curX = bfsQueue[front][0], curY = bfsQueue[front][1], curDis = bfsQueue[front][2];
        front++;
        if (curX == aimX && curY == aimY)
        {
            robotStep[robotId][0] = 2;
            robotStep[robotId][1] = curDis;
            return findNextStep(startX, startY, front - 1, robotId);
        }
        for (int i = 0; i < 4; i++)
        {
            int newX, newY;
            newX = curX + dx[seq[i]];
            newY = curY + dy[seq[i]];
            if (isValid(newX, newY) && !visitedMap[newX][newY])
            {
                // if (front <= 10 && robotMap[{newX, newY}] != 0 && robotMap[{newX, newY}] < robotId + 1)
                //     return -1;
                if (robotMap[{newX, newY}] == 0)
                {
                    visitedMap[newX][newY] = true;
                    bfsQueue[rear][0] = newX;
                    bfsQueue[rear][1] = newY;
                    bfsQueue[rear][2] = bfsQueue[front - 1][2] + 1;
                    bfsQueue[rear][3] = front - 1;
                    bfsQueue[rear][4] = seq[i];
                    rear++;
                }
            }
        }
    }
    return -1;
}
void toGoods(int robotId)
{
    our_map[robot[robotId].aimX][robot[robotId].aimY].goodRobotId = -1;
    pair<int, int> pos = findAimGood(robot[robotId].x, robot[robotId].y);
    robot[robotId].aimX = pos.first;
    robot[robotId].aimY = pos.second;
    our_map[robot[robotId].aimX][robot[robotId].aimY].goodRobotId = robotId;
}
void toBerth(int robotId)
{
    pair<int, int> pos = findAimBerth(robot[robotId].x, robot[robotId].y);
    robot[robotId].aimX = pos.first;
    robot[robotId].aimY = pos.second;
}
void robotInit(int robotId)
{
    toGoods(robotId);
}
int ttVal;
//void robotAction(int robotId)
//{
//    int curX = robot[robotId].x, curY = robot[robotId].y;
//    if (robot[robotId].goods && our_map[curX][curY].isBerth)
//    {
//        printf("pull %d\n", robotId);
//        // 更新泊位的货物队列。。。如果没pull成功呢？
//        int berthId = InWhichBerth(curX, curY); 
//        if (berthId != -1)
//        {
//            berth[berthId].goods.push_back(robot[robotId].goodValue);
//            ttVal += robot[robotId].goodValue;
//        }
//        toGoods(robotId);
//        robot[robotId].goods = 0;
//        robot[robotId].goodValue = 0; // 货物价值清零
//    }
//    else if ((!robot[robotId].goods) && curX == robot[robotId].aimX && curY == robot[robotId].aimY)
//    {
//        if (!our_map[curX][curY].GoodExist())
//        {
//            toGoods(robotId);
//        }
//        else
//        {
//           printf("get %d\n", robotId);
//            our_map[curX][curY].goodRobotId = -1;
//            toBerth(robotId);
//            robot[robotId].goods = 1;
//            robot[robotId].goodValue = our_map[curX][curY].goodValue;
//            our_map[curX][curY].isGoodExist = false;
//        }
//    }
//    int mov = robotBfsToAim(robot[robotId].x, robot[robotId].y, robot[robotId].aimX, robot[robotId].aimY, robotId);
//    if (mov != -1)
//    {
//       printf("move %d %d\n", robotId, mov);
//        robotMap[{curX, curY}] = 0;
//        robotMap[{curX + dx[mov], curY + dy[mov]}] = robotId + 1;
//        robot[robotId].x += dx[mov];
//        robot[robotId].y += dy[mov];
//        curX += dx[mov];
//        curY += dy[mov];
//    }
//    else
//    {
//        if (!robot[robotId].goods)
//            toGoods(robotId);
//        else
//            toBerth(robotId);
//    }
//    if (robot[robotId].goods && our_map[curX][curY].isBerth)
//    {
//       printf("pull %d\n", robotId);
//        // 更新泊位的货物队列
//        int berthId = InWhichBerth(curX, curY);
//        if (berthId != -1)
//        {
//            berth[berthId].goods.push_back(robot[robotId].goodValue);
//            ttVal += robot[robotId].goodValue;
//        }
//        toGoods(robotId);
//        robot[robotId].goods = 0;
//        robot[robotId].goodValue = 0; // 货物价值清零
//    }
//    else if ((!robot[robotId].goods) && curX == robot[robotId].aimX && curY == robot[robotId].aimY)
//    {
//        if (!our_map[curX][curY].GoodExist())
//        {
//            toGoods(robotId);
//        }
//        else
//        {
//           printf("get %d\n", robotId);
//            our_map[curX][curY].goodRobotId = -1;
//            toBerth(robotId);
//            robot[robotId].goods = 1;
//            robot[robotId].goodValue = our_map[curX][curY].goodValue;
//            our_map[curX][curY].isGoodExist = false;
//        }
//    }
//}
int pullGetFlag[11];
int getVal;//cout 
bool pullAndGet(int robotId){
    int curX = robot[robotId].x, curY = robot[robotId].y;
    if (robot[robotId].goods && our_map[curX][curY].isBerth)
    {
    	if(printRobot)
        printf("pull %d\n", robotId);
        // 更新泊位的货物队列。。。如果没pull成功呢？
        int berthId = InWhichBerth(curX, curY); 
        if (berthId != -1)
        {
            berth[berthId].goods.push_back(robot[robotId].goodValue);
            ttVal += robot[robotId].goodValue;
        }
        toGoods(robotId);
        robot[robotId].goods = 0;
        robot[robotId].goodValue = 0; // 货物价值清零
        return true;
    }
    else if ((!robot[robotId].goods) && curX == robot[robotId].aimX && curY == robot[robotId].aimY)
    {
        if (!our_map[curX][curY].GoodExist())
        {
            toGoods(robotId);
        }
        else
        {
        	if(printRobot)
           printf("get %d\n", robotId);
           getVal += our_map[curX][curY].goodValue;
            our_map[curX][curY].goodRobotId = -1;
            toBerth(robotId);
            robot[robotId].goods = 1;
            robot[robotId].goodValue = our_map[curX][curY].goodValue;
            our_map[curX][curY].isGoodExist = false;
            return true;
        }
    }
    return false;
}
bool nearRobot(int startX, int startY){
	int front = 0, rear = 0;
    map<pair<int, int>, int> visited;
    visited[{startX, startY}] = 1;
    bfsQueue[rear][0] = startX;
    bfsQueue[rear][1] = startY;
    bfsQueue[rear][2] = 0;
    rear++;
    while (front != rear)
    {
        int curX = bfsQueue[front][0], curY = bfsQueue[front][1], curDis = bfsQueue[front][2];
        if(curDis > 3) break;
        front++;
        for (int i = 0; i < 4; i++)
        {
            int newX, newY;
            newX = curX + dx[seq[i]];
            newY = curY + dy[seq[i]];
            if (isValid(newX, newY) && !visited[{newX, newY}])
            {
	            if(robotMap[{newX, newY}] != 0){
	            	return true;
				}
                visited[{newX, newY}]= 1;
                bfsQueue[rear][0] = newX;
                bfsQueue[rear][1] = newY;
                bfsQueue[rear][2] = bfsQueue[front - 1][2] + 1;
                rear++;
            }
        }
    }
    return false;
}
void toMove(int robotId){
    int mov, curX = robot[robotId].x, curY = robot[robotId].y;
	int robotFlag = nearRobot(curX, curY); 
//	cout<<robotFlag<<' ';
//	if(generateRandom() < 0.01 && !robot[robotId].goods){
//		toGoods(robotId);
//		mov = robotBfsToAim(robot[robotId].x, robot[robotId].y, robot[robotId].aimX, robot[robotId].aimY, robotId);
//	}
//	else 
	if(generateRandom() < 0.05 && robot[robotId].goods){
		toBerth(robotId);
		mov = robotBfsToAim(robot[robotId].x, robot[robotId].y, robot[robotId].aimX, robot[robotId].aimY, robotId);
	}
    else if(generateRandom() < 0.1 || pullGetFlag[robotId]|| robotFlag){
        mov = robotBfsToAim(robot[robotId].x, robot[robotId].y, robot[robotId].aimX, robot[robotId].aimY, robotId);
    }
    else{
        mov = robotPath[robotStep[robotId][0]][robotId];
        if(robotStep[robotId][0] > robotStep[robotId][1])
            mov = -1;
        robotStep[robotId][0]++;
    }
    if (mov != -1){
    	if(printRobot)
       printf("move %d %d\n", robotId, mov);
        robotMap[{curX, curY}] = 0;
        robotMap[{curX + dx[mov], curY + dy[mov]}] = robotId + 1;
        robot[robotId].x += dx[mov];
        robot[robotId].y += dy[mov];
    }
    else{
        if (!robot[robotId].goods)
            toGoods(robotId);
        else
            toBerth(robotId);
    }
}
void robotActionNew(int robotId){
    pullGetFlag[robotId] |= pullAndGet(robotId);
    toMove(robotId);
    pullGetFlag[robotId] = pullAndGet(robotId);
}
//
// end of robotaction
//


// start of the collision judgment

bool isAroundSafe (int robotId){    // 检测周围环境是否有碰撞可能
    switch(robotPath[robotStep[robotId][0]][robotId]){
        case -1:
            if((robotMap[{robot[robotId].x + dx[0] ,robot[robotId].y + dy[0]}] == 0) && (robotMap[{robot[robotId].x + dx[1] ,robot[robotId].y + dy[1]}] == 0) && (robotMap[{robot[robotId].x + dx[2] ,robot[robotId].y + dy[2]}] == 0) && (robotMap[{robot[robotId].x + dx[3] ,robot[robotId].y + dy[3]}] == 0)){return true;}
            else{return false;}
            break;
        case 0:
            if((robotMap[{robot[robotId].x ,robot[robotId].y + 2}] == 0) && (robotMap[{robot[robotId].x - 1 ,robot[robotId].y + 1}] == 0) && (robotMap[{robot[robotId].x + 1 ,robot[robotId].y + 1}] == 0)){return true;}
            else{return false;}
            break;
        case 1:
            if((robotMap[{robot[robotId].x ,robot[robotId].y - 2}] == 0) && (robotMap[{robot[robotId].x - 1 ,robot[robotId].y - 1}] == 0) && (robotMap[{robot[robotId].x + 1 ,robot[robotId].y - 1}] == 0)){return true;}
            else{return false;}
            break;
        case 2:
            if((robotMap[{robot[robotId].x - 2 ,robot[robotId].y}] == 0) && (robotMap[{robot[robotId].x - 1 ,robot[robotId].y - 1}] == 0) && (robotMap[{robot[robotId].x - 1,robot[robotId].y + 1}] == 0)){return true;}
            else{return false;}
            break;
        case 3:
            if((robotMap[{robot[robotId].x + 2 ,robot[robotId].y}] == 0) && (robotMap[{robot[robotId].x + 1,robot[robotId].y - 1}] == 0) && (robotMap[{robot[robotId].x + 1 ,robot[robotId].y + 1}] == 0)){return true;}
            else{return false;}
            break;

    }
}
void swap(int & a, int & b){
    int tmp = a;
    a = b;
    b = tmp;
}
void StraightHit(int robotId_1, int robotId_2){  // 双方单路直撞
    if(robot[robotId_1].goods == 0 && robot[robotId_2].goods == 0){
        swap(robot[robotId_1].aimX, robot[robotId_2].aimX);
        swap(robot[robotId_1].aimY, robot[robotId_2].aimY);
    }
    else{
        if(robotId_1 > robotId_2){
            //robot2让路
        }
        else{
            //robot1让路
        }
    }
}

void SideHit(int robotId_1, int robotId_2){
    if(robotId_1 > robotId_2){
        //robot2暂停
    }
    else{
        //robot1暂停
    }
}


// end of the collision judgment

// 船的specific_status
#define WAIT 3
#define LOAD 4
#define DONE 5
#define TO_BERTH 6
#define TO_VIRTUAL 7
#define BERTH_TO_BERTH 8

struct Boat
{
    // 船的容积是一样的
    int num;             // 船上物品的数量
    int goods_value;     // 船上物品的价值
    int pos;             // 目标泊位，-1表示虚拟点
    int status;          // 0表示移动（从地图上消失），1表示装货状态或运输完成，2表示等待
    int specific_status; // WAIT表示等待，LOAD表示装货，DONE表示运输完成，TO_BERTH表示去泊位，TO_VIRTUAL表示去虚拟点，BERTH_TO_BERTH改变泊位
    int load_time;       // 在泊位上装货/等待的时间
    int empty_time;      // 在泊位上没东西装的时间
    Boat()
    {
        num = 0;
        goods_value = 0;
        pos = -1;
        status = 1;
        specific_status = DONE;
        load_time = 0;
    }
} boat[10];

void Init()
{
    for (int i = 1; i <= n; i++)
        scanf("%s", ch[i] + 1);
    for (int i = 0; i < berth_num; i++)
    {
        scanf("%d", &id);
        scanf("%d%d%d%d", &berth[id].x, &berth[id].y, &berth[id].transport_time, &berth[id].loading_speed);
        averageTransportTime += berth[id].transport_time;
    }
    averageTransportTime /= 10;
    scanf("%d", &boat_capacity);
    char okk[100];
    scanf("%s", okk);
    printf("OK\n");
    fflush(stdout);
}

// 初始化地图类
void mapInit()
{
    for (int i = 0; i < 200; i++)
    {
        for (int j = 0; j < 200; j++)
        {
            our_map[i][j].isBarrierExist = (ch[i + 1][j + 1] == '*' || ch[i + 1][j + 1] == '#');
            our_map[i][j].isRobotExist = (ch[i + 1][j + 1] == 'A');
            our_map[i][j].isBerth = (ch[i + 1][j + 1] == 'B');
        }
    }
}

/**
 * \brief 每一帧判题器告诉场面信息后，更新boat上的变量
 * 维护船只的具体状态，状态转变
 * WAIT表示等待，LOAD表示装货，DONE表示运输完成，TO_BERTH表示去泊位，TO_VIRTUAL表示去虚拟点；
 * 以及船只目前装载的货物的数量和价值，泊位的货物队列
 */
void UpdateBoatSpecificState(int boat_id)
{
	if(closedBerth == 10) return;
    if (boat[boat_id].status == 1)
    {
        if (boat[boat_id].specific_status == TO_BERTH)
        {
            boat[boat_id].num = 0;
            boat[boat_id].goods_value = 0;
            boat[boat_id].specific_status = LOAD;
            boat[boat_id].load_time = 0;
            boat[boat_id].empty_time = 0;
        }
        else if (boat[boat_id].specific_status == WAIT || boat[boat_id].specific_status == BERTH_TO_BERTH)
        {
            boat[boat_id].specific_status = LOAD;
            boat[boat_id].load_time = 0;
            boat[boat_id].empty_time = 0;
        }
        else if (boat[boat_id].specific_status == TO_VIRTUAL)
        {
            boat[boat_id].num = 0;
            boat[boat_id].goods_value = 0;
            boat[boat_id].specific_status = DONE;
            boat[boat_id].load_time = 0;
            boat[boat_id].empty_time = 0;
        }
    }
    else if (boat[boat_id].status == 2)
    {
        boat[boat_id].specific_status = WAIT;
        boat[boat_id].load_time = 0;
        boat[boat_id].empty_time = 0;
    }
    if (boat[boat_id].specific_status == LOAD)
    {
        berth[boat[boat_id].pos].is_occupied = true;
        boat[boat_id].load_time++; // 在泊位上装货/等待的时间++
        for (int i = 0; i < berth[boat[boat_id].pos].loading_speed; i++)
        {
            if (berth[boat[boat_id].pos].goods.empty())
            {
                boat[boat_id].empty_time++;
                break;
            }

            boat[boat_id].goods_value += berth[boat[boat_id].pos].goods.front();
            berth[boat[boat_id].pos].goods.pop_front();
            boat[boat_id].num += 1;
        }
    }
}

int Input()
{
    scanf("%d%d", &id, &money);

    int num;
    scanf("%d", &num);
    for (int i = 1; i <= num; i++)
    {
        int x, y, val;
        scanf("%d%d%d", &x, &y, &val);
        our_map[x][y].generateGood(val, id); // 生成货物
    }
    robotMap.clear();
    for (int i = 0; i < robot_num; i++)
    {
        int sts;
        scanf("%d%d%d%d", &robot[i].goods, &robot[i].x, &robot[i].y, &sts);
        robotMap[{robot[i].x, robot[i].y}] = i + 1;
    }
    for (int i = 0; i < 5; i++)
        scanf("%d%d\n", &boat[i].status, &boat[i].pos);

    // 维护船只的具体状态........
    for (int i = 0; i < 5; i++)
        UpdateBoatSpecificState(i);

    char okk[100];
    scanf("%s", okk);
    return id;
}

/**
 * \brief 计算目前泊位上船只可装下的货物的总价值
 */
int GetTotalValue(int berth_id)
{
    int total_value = 0; // 能装载的货物的总价值
    int cnt = 0;         // 目前装载的货物的数量
    while (cnt < boat_capacity && cnt < berth[berth_id].goods.size())
    {
        if (berth[berth_id].goods.empty())
            break;
        total_value += berth[berth_id].goods[cnt];
        cnt++;
    }
    return total_value;
}

// 超参数
int total_value_weight = 100;
int transport_time_weight = 10;
int is_occupied_weight = 1000;
int loading_speed_weight = 1000;

/**
 * \brief 需要考虑如何计算泊位的优先级......
 */
int GetBerthPriority(int berth_id)
{
    int priority = 0;
    int total_value = GetTotalValue(berth_id);
    // 计算泊位的优先级
    berthVal[berth_id] = total_value;
    if (berth[berth_id].is_occupied)
    {
        priority = -1000000000;
    }
    else
    {
        priority = total_value * total_value_weight - berth[berth_id].transport_time * transport_time_weight + berth[berth_id].loading_speed * loading_speed_weight;
    }
    return priority;
}

/**
 * \brief 获取船只的目标泊位
 */
void printBerthInf(){
    cout<<id<<' '<<money<<' '<<ttVal<<endl;
    for (int i = 0; i < berth_num; i++)
    {
        berthVal[i] = GetTotalValue(i);
    }
	for(int i=0;i<10;i++)
		cout<<berthVal[i]<<' ';
	cout<<endl;
	for(int i=0;i<10;i++)
		cout<<berth[i].is_occupied<<' ';
	cout<<endl;
	for(int i=0;i<10;i++)
		cout<<berth[i].is_closed<<' ';
	cout<<endl;
}
int GetBerthId()
{
    int priority[berth_num + 10]; // 计算泊位的优先级，数值越大优先级越高
    int tmp_priority;
    for (int i = 0; i < berth_num; i++)
    {
        tmp_priority = GetBerthPriority(i);
        priority[i] = tmp_priority;
    }
    // 选择优先级最高的泊位
    int max_priority = -999999;
    int max_priority_id = -1;
    for (int i = 0; i < berth_num; i++)
    {
        if (priority[i] > max_priority && !berth[i].is_closed)
        {
            max_priority = priority[i];
            max_priority_id = i;
        }
    }
    berth[max_priority_id].is_occupied = true;
    return max_priority_id;
}

/**
 * \brief 装了多少货物后，船只离开泊位进行运输，需要考虑怎么设置阈值.......
 */
/*
int GetThreshold(int boat_id)
{
   // int threshold = 50;
   int threshold = berth[boat[boat_id].pos].transport_time / 2;
   return threshold;
}
*/
bool BoatReadyGo(int boat_id)
{

    // int threshold = GetThreshold(boat_id);
    //  船只满载或者价值超过阈值

    if (15000 - id <= berth[boat[boat_id].pos].transport_time * 1.1)
    {
    	berth[boat[boat_id].pos].is_closed = 1;
	    closedBerth++;
        return true;
    }
    if(15000 - id <= berth[boat[boat_id].pos].transport_time * 3){    
		if(boat[boat_id].num >= boat_capacity){
	    	if(printBerth)
	    	cout<<boat[boat_id].pos<<' '<<"FULL"<<endl;
	    	berth[boat[boat_id].pos].is_closed = 1;
	    	closedBerth++;
	    	return true;
		}
    	return false;
	}
    // if(15000 - id <=  3 * 15000 - id > berth[boat[boat_id].pos].transport_time && 15000 - id > berth[boat[boat_id].pos].transport_time )
    if ((boat[boat_id].empty_time < 50 && boat[boat_id].num < boat_capacity) || (boat[boat_id].num < 3 / 4.0 * boat_capacity && 15000 - id >= berth[boat[boat_id].pos].transport_time + averageTransportTime * 2.5))
    {
        return false;
    }
    else
    {
    	if(15000 - id <= berth[boat[boat_id].pos].transport_time * 4 && closedBerth < 5){
    		berth[boat[boat_id].pos].is_closed = true;
    		closedBerth ++;
		}
        return true;
    }
}
bool ChangeBerth(int boat_id)
{
    if (boat[boat_id].empty_time > 50 && boat[boat_id].num < 3 / 4.0 * boat_capacity && 15000 - id >= berth[boat[boat_id].pos].transport_time + averageTransportTime * 2.5)
    {
    	if(15000 - id <= berth[boat[boat_id].pos].transport_time * 4 && closedBerth < 5){
    		berth[boat[boat_id].pos].is_closed = true;
    		closedBerth ++;
		}
        return true;
    }
    else
    {
        return false;
    }
}
/**
 * \brief 只考虑了在虚拟点时需下达命令，在泊位装货时需下达命令.......
 */
void GiveBoatCommand()
{
    for (int i = 0; i < 5; i++)
    {
        //        printf("boat %d : specific_status is %d,good num is %d,good value is %d,pos is %d,load_time is %d\n", i, boat[i].specific_status, boat[i].num, boat[i].goods_value, boat[i].pos, boat[i].load_time);
        if (boat[i].specific_status == DONE)
        { // 如果船只运输完成，在虚拟点
            boat[i].num = 0;
            boat[i].goods_value = 0;
            boat[i].load_time = 0;
            boat[i].specific_status = TO_BERTH;
            if(printBerth){
            	printBerthInf();
			}
            printf("ship %d %d\n", i, GetBerthId());
            continue;
        }
        if (boat[i].specific_status == LOAD)
        {
            // 如果船只在泊位装货
            if (BoatReadyGo(i))
            {
		        if(printBerth){
		        	printBerthInf();
				}
                printf("go %d\n", i);
                boat[i].specific_status = TO_VIRTUAL;
                berth[boat[i].pos].is_occupied = false;
                continue;
            }
            if (ChangeBerth(i))
            {
            	if(printBerth){
		        	printBerthInf();
				}
                printf("ship %d %d\n", i, GetBerthId());
                boat[i].specific_status = BERTH_TO_BERTH;
                berth[boat[i].pos].is_occupied = false;
            }
        }
    }
}
int main()
{
//             freopen("output.txt", "r", stdin);
//             freopen("myoutputship.txt", "w", stdout);
    // srand(static_cast<unsigned int>(time(0)));cout
//    printRobot = 0;
//    printBerth = 1;
    Init();
    mapInit(); // 地图类初始化
    initDisBerth();
    for (int zhen = 1; zhen <= 15000; zhen++)
    {
        id = Input();
//     	cout<<id<<' '<<money<<' '<<ttVal<<' '<<getVal<<endl;
		for(int i = 0; i<9;i++)
        if (zhen == 1)
            for (int i = 0; i < robot_num; i++)
            {
                robotInit(i);
            }
        for (int i = 0; i < robot_num; i++)
            // 为机器人下达命令........
            //             printf("move %d %d\n", i, rand() % 4);
//            robotAction(i);
			robotActionNew(i);
//		cout<<endl;
        // 为船只下达命令........
        GiveBoatCommand();
        if(printRobot) 
        puts("OK");
        fflush(stdout);
    }
    return 0;
}

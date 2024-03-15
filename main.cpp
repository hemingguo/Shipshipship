//#include <unordered_map>
//#include <map>
//#include <queue>
//#include <utility>
//#include <iostream>
 #include <bits/stdc++.h>
using namespace std;

const int n = 200;
const int robot_num = 10;
const int berth_num = 10;
const int N = 210;


int money, boat_capacity, id;
char ch[N][N]; // 字符地图
int gds[N][N];

struct Square
{
    bool isBarrierExist;          // 是否有障碍物
    bool isGoodExist;             // 是否有货物
    bool isRobotExist;            // 是否有机器人
    bool isBerth;            // 是否是泊位
    int goodValue;                // 货物价值
    int goodTime;                 // 货物存留时间
    int goodRobotId;
//    unordered_map<int, int> sign; // 该格子被标记的路径(经过该格子第几帧<-->机器人编号)
    Square() : goodRobotId(-1), isBarrierExist(false), isGoodExist(false), isRobotExist(false), isBerth(false), goodTime(0), goodValue(0)
    {
    }
    void generateGood() // 货物生成
    {
        isGoodExist = true;
        goodTime = 1000;
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
    bool GoodExist(){
        return (id - generateTime <= 1000) && isGoodExist;
    }
};

Square our_map[n][n]; // 创建地图


struct Robot
{
    int x, y, goods; // 是否携带物品
    int status;      // 0表示恢复，1表示正常运行
    int mbx, mby;    // 此帧机器人所在的位置
    int aimX, aimY;     //目标位置（added by cyh）
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
    queue<int> goods; // 待装载的货物
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
    }
} berth[berth_num + 10];




map<pair<int, int>, bool> robotMap;//(x, y), haverobot
//
//robotaction by cyh
//
int dx[4] = {0, 0, -1, 1};
int dy[4] = {1, -1, 0, 0};
int disBerth[205][205];


bool isValid(int x, int y){
    return (x >= 0 && x < 200 && y >=0 && y < 200 && (!our_map[x][y].isBarrierExist));
}
void berthBfs(int x, int y){
    queue<pair<int, int> > q;
    q.push({x, y});
    disBerth[x][y] = 0;
    while (!q.empty()) {
        pair<int, int> curr = q.front();
        q.pop();
        int curX = curr.first, curY = curr.second;
        for (int i = 0; i < 4; i++) {
            int newX = curX + dx[i];
            int newY = curY + dy[i];
            if(isValid(newX, newY) && disBerth[curX][curY] + 1 < disBerth[newX][newY]){
                q.push({newX, newY});
                disBerth[newX][newY] = disBerth[curX][curY] + 1;
            }
        }
    }
}
void initDisBerth(){
    for(int i = 0; i < 200; i++)
        for(int j = 0; j < 200; j++)
            disBerth[i][j] = 45000;
    for(int i = 0; i < berth_num; i++)
    {
        berthBfs(berth[i].x, berth[i].y);
    }
}
float valueFunctionGood(int dis, int goodX, int goodY, int value){
    return value/(float)(dis + disBerth[goodX][goodY]);
}
float maxW;
pair<int, int> maxGoodPos;
bool visitedMap[205][205];
int robotPath [45000][11][2];
int bfsQueue[45000][4];//x,y,step,lastStep;

void findAimGood(int startX, int startY){
    int front = 0, rear = 0;
    for(int i = 0; i < 200; i++)
        for(int j = 0; j < 200; j++)
            visitedMap[i][j] = false;
    visitedMap[startX][startY] = true;
    bfsQueue[rear][0] = startX;
    bfsQueue[rear][1] = startY;
    bfsQueue[rear][2] = 0;
    rear++;
    visitedMap[startX][startY] = true;
//    cout<<startX<<' '<<startY<<"START"<<endl;
    while (front != rear) {
        int curX = bfsQueue[front][0], curY = bfsQueue[front][1], curDis = bfsQueue[front][2];

        front++;
        if (our_map[curX][curY].GoodExist() && our_map[curX][curY].goodRobotId == -1 && (!our_map[curX][curY].isBerth) && valueFunctionGood(curDis, curX, curY, our_map[curX][curY].goodValue) > maxW) {
            maxW = valueFunctionGood(curDis, curX, curY, our_map[curX][curY].goodValue);
            maxGoodPos = {curX, curY};
        }
        for (int i = 0; i < 4; i++) {
            int newX = curX + dx[i];
            int newY = curY + dy[i];
            if (isValid(newX, newY) && !visitedMap[newX][newY]) {
                visitedMap[newX][newY] = true;
                bfsQueue[rear][0] = newX;
                bfsQueue[rear][1] = newY;
                bfsQueue[rear][2] = curDis + 1;
                rear++;
            }
        }
    }
}
pair<int, int> findAimBerth(int startX, int startY){
    int front = 0, rear = 0;
    if(our_map[startX][startY].isBerth)
        return {startX, startY};
    for(int i = 0; i < 200; i++)
        for(int j = 0; j < 200; j++)
            visitedMap[i][j] = false;
    visitedMap[startX][startY] = true;
    bfsQueue[rear][0] = startX;
    bfsQueue[rear][1] = startY;
    bfsQueue[rear][2] = 0;
    rear++;
    visitedMap[startX][startY] = true;
    while (front != rear) {
        int curX = bfsQueue[front][0], curY = bfsQueue[front][1], curDis = bfsQueue[front][2];
        front++;
        for (int i = 0; i < 4; i++) {
            int newX = curX + dx[i];
            int newY = curY + dy[i];
            if (isValid(newX, newY) && !visitedMap[newX][newY]) {
                if(our_map[newX][newY].isBerth)
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
int findNextStep(int startX, int startY, int bfsId){
    int lastId = bfsQueue[bfsId][3];
//    	cout<<"FIND@"<<endl;
    if(bfsQueue[lastId][0] == startX && bfsQueue[lastId][1] == startY){
        for(int i = 0; i < 4; i++)
            if(bfsQueue[bfsId][0] - startX == dx[i] && bfsQueue[bfsId][1] - startY == dy[i])
                return i;
    }
    return findNextStep(startX, startY, lastId);
}
int robotBfsToAim(int startX, int startY, int aimX, int aimY){
    int front = 0, rear = 0;
    if(startX == aimX && startY == aimY)
        return -1;
    for(int i = 0; i < 200; i++)
        for(int j = 0; j < 200; j++)
            visitedMap[i][j] = false;
    visitedMap[startX][startY] = true;
    bfsQueue[rear][0] = startX;
    bfsQueue[rear][1] = startY;
    bfsQueue[rear][2] = 0;
    bfsQueue[rear][3] = 0;
    rear++;
    visitedMap[startX][startY] = true;
    while (front != rear) {
        int curX = bfsQueue[front][0], curY = bfsQueue[front][1], curDis = bfsQueue[front][2];
//		        cout<<curX<<' '<<curY<<' '<<curDis<<endl;
        front++;
        for (int i = 0; i < 4; i++) {
            int newX = curX + dx[i];
            int newY = curY + dy[i];
            if (isValid(newX, newY) && !visitedMap[newX][newY] && (!robotMap[{newX, newY}])) {
                if(curX == aimX && curY == aimY){
//                	cout<<"FIND!"<<endl;
                    return findNextStep(startX, startY, front - 1);
				}
                visitedMap[newX][newY] = true;
                bfsQueue[rear][0] = newX;
                bfsQueue[rear][1] = newY;
                bfsQueue[rear][2] = bfsQueue[front][2] + 1;
                bfsQueue[rear][3] = front - 1;
                rear++;
            }
        }
    }
    return -1;
}
void robotInit(int robotId){
	maxW = 0;
    robot[robotId].goods = 0;
    maxGoodPos = {robot[robotId].x, robot[robotId].y};
    findAimGood(robot[robotId].x, robot[robotId].y);
    if(maxW > 0){
        robot[robotId].aimX = maxGoodPos.first;
        robot[robotId].aimY = maxGoodPos.second;
    }
//    cout<<maxW<<' '<<robotId<<' '<<robot[robotId].aimX<<' '<<robot[robotId].aimY<<endl;
} 
void robotAction(int state, int robotId){
    int curX = robot[robotId].x, curY = robot[robotId].y;
    if(robot[robotId].goods && our_map[curX][curY].isBerth){
        printf("pull %d\n", robotId);
        maxW = 0;
        robot[robotId].goods = 0;
        findAimGood(robot[robotId].x, robot[robotId].y);
//        if(maxW > 0){
            robot[robotId].aimX = maxGoodPos.first;
            robot[robotId].aimY = maxGoodPos.second;
            our_map[robot[robotId].x][robot[robotId].y].goodRobotId = -1;
            our_map[maxGoodPos.first][maxGoodPos.second].goodRobotId = robotId;
//        }
    }
    else if((!robot[robotId].goods) && curX == robot[robotId].aimX && curY == robot[robotId].aimY ){
    	if(!our_map[curX][curY].GoodExist()) {
            maxW = 0;
            robot[robotId].goods = 0;
            findAimGood(robot[robotId].x, robot[robotId].y);
            if(maxW > 0){
                robot[robotId].aimX = maxGoodPos.first;
                robot[robotId].aimY = maxGoodPos.second;
            }
        }
        else{
            printf("get %d\n", robotId);
            pair<int, int> pos = findAimBerth(curX, curY);
            robot[robotId].aimX = pos.first;
            robot[robotId].aimY = pos.second;
            robot[robotId].goods = our_map[curX][curY].goodValue;
            our_map[curX][curY].isGoodExist = false;
        }

    }
//    cout<<"TOCALAULATE"<<endl;
    int mov = robotBfsToAim(robot[robotId].x, robot[robotId].y, robot[robotId].aimX, robot[robotId].aimY);
//    cout<<robotId<<' '<<robot[robotId].aimX<<' '<<robot[robotId].aimY<<' '<<mov<<endl; 
    if(mov != -1){
        printf("move %d %d\n", robotId, mov);
        robotMap[{curX, curY}] = 0;
        robotMap[{curX + dx[mov], curY + dx[mov]}] = 1;
        curX += dx[mov];
        curY += dy[mov];
    }
    if(robot[robotId].goods && our_map[curX][curY].isBerth){
        printf("pull %d\n", robotId);
        maxW = 0;
        robot[robotId].goods = 0;
        findAimGood(robot[robotId].x, robot[robotId].y);
        if(maxW > 0){
            robot[robotId].aimX = maxGoodPos.first;
            robot[robotId].aimY = maxGoodPos.second;
        }
    }
    else if((!robot[robotId].goods) && curX == robot[robotId].aimX && curY == robot[robotId].aimY){
    	if(!our_map[curX][curY].GoodExist()) {
            maxW = 0;
            robot[robotId].goods = 0;
            findAimGood(curX, curX);
//            if(maxW > 0){
                robot[robotId].aimX = maxGoodPos.first;
                robot[robotId].aimY = maxGoodPos.second;
                
            our_map[robot[robotId].x][robot[robotId].y].goodRobotId = -1;
            our_map[maxGoodPos.first][maxGoodPos.second].goodRobotId = robotId;
//            }
        }
        else{
            printf("get %d\n", robotId);
            pair<int, int> pos = findAimBerth(curX, curY);
            robot[robotId].aimX = pos.first;
            robot[robotId].aimY = pos.second;
            robot[robotId].goods = our_map[curX][curY].goodValue;
            our_map[curX][curY].isGoodExist = false;
        }
    }
}
//
//end of robotaction
//

// 船的specific_status
#define WAIT 3
#define LOAD 4
#define DONE 5
#define TO_BERTH 6
#define TO_VIRTUAL 7

struct Boat
{
    // 船的容积是一样的
    int num;             // 船上物品的数量
    int goods_value;     // 船上物品的价值
    int pos;             // 目标泊位，-1表示虚拟点
    int status;          // 0表示移动（从地图上消失），1表示装货状态或运输完成，2表示等待
    int specific_status; // WAIT表示等待，LOAD表示装货，DONE表示运输完成，TO_BERTH表示去泊位，TO_VIRTUAL表示去虚拟点
    Boat()
    {
        num = 0;
        goods_value = 0;
        specific_status = DONE;
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
    }
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
 * \brief 维护船只的具体状态，
 * WAIT表示等待，LOAD表示装货，DONE表示运输完成，TO_BERTH表示去泊位，TO_VIRTUAL表示去虚拟点；
 * 以及船只目前装载的货物的数量和价值，泊位的货物队列
 */
void UpdateBoatSpecificState(int boat_id)
{
    if (boat[boat_id].status == 1)
    {
        if (boat[boat_id].specific_status == TO_BERTH || boat[boat_id].specific_status == WAIT)
        {
            boat[boat_id].specific_status = LOAD;
        }
        else if (boat[boat_id].specific_status == TO_VIRTUAL)
        {
            boat[boat_id].specific_status = DONE;
        }
    }
    else if (boat[boat_id].status == 2)
    {
        boat[boat_id].specific_status = WAIT;
    }
    if (boat[boat_id].specific_status == LOAD)
    {
        for (int i = 0; i < berth[boat[boat_id].pos].loading_speed; i++)
        {
            if (berth[boat[boat_id].pos].goods.empty())
                break;
            boat[boat_id].goods_value += berth[boat[boat_id].pos].goods.front();
            berth[boat[boat_id].pos].goods.pop();
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
        our_map[x][y].generateGood(); // 生成货物

    }
    for (int i = 0; i < robot_num; i++)
    {
        int sts;
        scanf("%d%d%d%d", &robot[i].goods, &robot[i].x, &robot[i].y, &sts);
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
        int tmp = berth[berth_id].goods.front();
        total_value += tmp;
        berth[berth_id].goods.pop();
        berth[berth_id].goods.push(tmp);
        cnt++;
    }
    return total_value;
}

// 超参数
int total_value_weight = 100;
int transport_time_weight = 10;
int is_occupied_weight = 1000;
int loading_speed_weight = 1;

/**
 * \brief 需要考虑如何计算泊位的优先级......
 */
int GetBerthPriority(int berth_id)
{
    int priority = 0;
    int total_value = GetTotalValue(berth_id);
    // 计算泊位的优先级
    if (berth[berth_id].is_occupied)
    {
        priority = total_value * total_value_weight - berth[berth_id].transport_time * transport_time_weight - is_occupied_weight - berth[berth_id].loading_speed * loading_speed_weight;
    }
    else
    {
        priority = total_value * total_value_weight - berth[berth_id].transport_time * transport_time_weight - berth[berth_id].loading_speed * loading_speed_weight;
    }
    return priority;
}

/**
 * \brief 获取船只的目标泊位
 */
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
    int max_priority = -99999;
    int max_priority_id = -1;
    for (int i = 0; i < berth_num; i++)
    {
        if (priority[i] > max_priority)
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
int GetThreshold(int boat_id)
{
    int threshold = berth[boat[boat_id].pos].transport_time;
    return threshold;
}

bool BoatReadyGo(int boat_id)
{
    int threshold = GetThreshold(boat_id);
    // 船只满载或者价值超过阈值
    if (boat[boat_id].num >= boat_capacity || boat[boat_id].goods_value >= threshold)
    {
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
        if (boat[i].specific_status == DONE)
        { // 如果船只运输完成，在虚拟点
            boat[i].specific_status = TO_BERTH;
            printf("ship %d %d\n", i, GetBerthId());
            continue;
        }
        if (boat[i].specific_status == LOAD)
        {
            // 如果船只在泊位装货
            if (BoatReadyGo(i))
            {
                boat[i].specific_status = TO_VIRTUAL;
                printf("go %d\n", i);
            }
            continue;
        }
    }
}
int main()
{
//     freopen("output.txt", "r", stdin);
//     freopen("myoutput.txt", "w", stdout);
    Init();
    //added by cyh
    mapInit(); // 地图类初始化
    initDisBerth();
    //
    for (int zhen = 1; zhen <= 15000; zhen++)
    {
        id = Input();
        if(zhen == 1)
        	for (int i = 0; i < robot_num; i++)
        	{
//        		cout<<robot[i].x<<' '<<robot[i].y<<endl;
        		robotInit(i);
        		
			}
        for (int i = 0; i < robot_num; i++)
            // 为机器人下达命令........
//             printf("move %d %d\n", i, rand() % 4);
            robotAction(1, i);
        // 为船只下达命令........
        void GiveBoatCommand();
        puts("OK");
        fflush(stdout);
    }

    return 0;
}

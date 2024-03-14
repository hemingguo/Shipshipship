#include <bits/stdc++.h>

using namespace std;

const int n = 200;
const int robot_num = 10;
const int berth_num = 10;
const int N = 210;

struct Square
{
    bool isBarrierExist;          // 是否有障碍物
    bool isGoodExist;             // 是否有货物
    bool isRobotExist;            // 是否有机器人
    bool isBerthExist;            // 是否是泊位
    int goodTime;                 // 货物存留时间
    unordered_map<int, int> sign; // 该格子被标记的路径(经过该格子第几帧<-->机器人编号)
    Square() : isBarrierExist(false), isGoodExist(false), isRobotExist(false), isBerthExist(false), goodTime(0)
    {
    }
    void generateGood() // 货物生成
    {
        isGoodExist = true;
        goodTime = 1000;
    }
    void testGood() // 货物存留计时器
    {
        if (isGoodExist)
        {
            goodTime--;
            if (goodTime == 0)
            {
                isGoodExist = false;
            }
        }
    }
};

Square our_map[n][n]; // 创建地图

struct Robot
{
    int x, y, goods;
    int status;
    int mbx, mby;
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
    Berth() {}
    Berth(int x, int y, int transport_time, int loading_speed)
    {
        this->x = x;
        this->y = y;
        this->transport_time = transport_time;
        this->loading_speed = loading_speed;
    }
} berth[berth_num + 10];

struct Boat
{
    int num, pos, status;
} boat[10];

int money, boat_capacity, id;
char ch[N][N];
int gds[N][N];

void Init()
{
    for (int i = 1; i <= n; i++)
        scanf("%s", ch[i] + 1);
    for (int i = 0; i < berth_num; i++)
    {
        int id;
        scanf("%d", &id);
        scanf("%d%d%d%d", &berth[id].x, &berth[id].y, &berth[id].transport_time, &berth[id].loading_speed);
    }
    scanf("%d", &boat_capacity);
    char okk[100];
    scanf("%s", okk);
    printf("OK\n");
    fflush(stdout);
}

void mapInit() // 初始化地图类
{
    for (int i = 0; i < 200; i++)
    {
        for (int j = 0; j < 200; j++)
        {
            our_map[i][j].isBarrierExist = ch[i + 2][j] == '*' or ch[i + 2][j] == '#' ? true : false;
            our_map[i][j].isRobotExist = ch[i + 2][j] == 'A' ? true : false;
            our_map[i][j].isBerthExist = ch[i + 2][j] == 'B' ? true : false;
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
    char okk[100];
    scanf("%s", okk);
    return id;
}

int main()
{
    Init();
    mapInit(); // 地图类初始化
    for (int zhen = 1; zhen <= 15000; zhen++)
    {
        int id = Input();

        //....

        for (int i = 0; i < robot_num; i++)
            printf("move %d %d\n", i, rand() % 4);

        puts("OK");
        fflush(stdout);
    }

    return 0;
}

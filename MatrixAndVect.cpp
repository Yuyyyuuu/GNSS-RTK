#include"MyHeadFile.h"

/*
**********************************************************************************
函数名：向量点积
参数：m 数组A的长度
      n 数组B的长度
      A 第一个向量的数组表示
      B 第二个向量的数组表示
返回值：两个向量的点积结果（数值）

函数功能：计算向量A与B的点积，并以返回值的形式传出
注：该函数要求向量A和向量B的长度大于零且相同，否则无法进行运算，返回0.0表示失败。
**********************************************************************************
*/
double VectDot(int m, int n, const double A[], const double B[])
{
    // 首先判断数组长度是否合法且匹配
    if (m <= 0 || n <= 0 || m != n)
    {
        // 这里简单地返回0.0作为错误值
        printf("Error dimension in VectDot!\n");
        return 0.0;
    }

    // 计算向量点积
    double DotResult = 0;
    for (int i = 0; i < m; i++)
    {
        DotResult = DotResult + A[i] * B[i];
    }

    return DotResult;
}

/*
**********************************************************************************
函数名：向量叉乘
参数：m 数组A的长度
      n 数组B的长度
      A 第一个向量的数组表示
      B 第二个向量的数组表示
      C 存储向量叉乘结果的数组
返回值：是否成功进行叉乘运算

函数功能：计算两个长度为3的向量A和B的叉乘结果，并将结果存储在C数组中。
注：该函数要求向量A和向量B的长度都为3，否则无法进行叉乘运算，返回false表示失败。
**********************************************************************************
*/
bool CrossDot(int m, int n, const double A[], const double B[], double C[])
{
    // 首先判断向量的维度是否正确
    if (m != 3 || n != 3)
    {
        // 如果向量维度不正确，返回错误值
        printf("Error dimension in CrossDot!\n");
        return false;
    }

    // 计算叉乘结果向量
    C[0] = A[1] * B[2] - A[2] * B[1];
    C[1] = A[2] * B[0] - A[0] * B[2];
    C[2] = A[0] * B[1] - A[1] * B[0];

    return true;
}

/*
*******************************************************
函数名：矩阵相加
参数：m  矩阵的行数(默认两个矩阵的行数相等)
      n  矩阵的列数(默认两个矩阵的列数相等)
      M1 第一个矩阵的数组表示
      M2 第二个矩阵的数组表示
      M3 存储矩阵相加结果的数组
返回值：是否成功进行矩阵相加运算

函数功能：计算两个矩阵的相加，将结果存储在M3数组中。
注：该函数要求矩阵的行数和列数都必须大于0，
    否则无法进行相加运算，返回false表示失败。
*******************************************************
*/
bool MatrixAddition(int m, int n, const double M1[], const double M2[], double M3[])
{
    // 首先判断矩阵的行数和列数是否合法
    if (m <= 0 || n <= 0)
    {
        // 如果行数或列数小于等于0，返回错误值
        printf("Error dimension in MatrixAddition!\n");
        return false;
    }

    // 执行矩阵相加运算
    for (int i = 0; i < m; i++)
    {
        for (int j = 0; j < n; j++)
        {
            int index = i * n + j;
            M3[index] = M1[index] + M2[index];
        }
    }

    return true;
}

/*
********************************************************
函数名：矩阵减法
参数：m  矩阵的行数(默认两个矩阵的行数相等)
      n  矩阵的列数(默认两个矩阵的列数相等)
      M1 第一个矩阵的数组表示
      M2 第二个矩阵的数组表示
      M3 存储矩阵相减结果的数组
返回值：是否成功进行矩阵相减运算

函数功能：计算两个矩阵的相减，将结果存储在M3数组中。
注：该函数要求矩阵的行数和列数都必须大于0，
    否则无法进行相减运算，返回false表示失败。
********************************************************
*/
bool MatrixSubtraction(int m, int n, const double M1[], const double M2[], double M3[])
{
    // 首先判断矩阵的行数和列数是否合法
    if (m <= 0 || n <= 0)
    {
        // 如果行数或列数小于等于0，返回错误值
        printf("Error dimension in MatrixSubtraction!\n");
        return false;
    }
    // 执行矩阵相减运算
    for (int i = 0; i < m; i++)
    {
        for (int j = 0; j < n; j++)
        {
            int index = i * n + j;
            M3[index] = M1[index] - M2[index];
        }
    }

    return true;
}

/*
***************************************************************************
函数名：矩阵相乘
参数：m1 M1矩阵的行数
      n1 M1矩阵的列数
      m2 M2矩阵的行数
      n2 M2矩阵的列数
      M1 第一个矩阵的数组表示
      M2 第二个矩阵的数组表示
      M3 存储矩阵相乘结果的数组
返回值：是否成功进行矩阵相乘运算

函数功能：计算两个矩阵的相乘，将结果存储在M3数组中。
注：该函数要求矩阵的行数和列数都必须大于0，
    要求M1的列数等于M2的行数，
    才能进行矩阵相乘运算。否则无法进行相乘运算，返回false表示失败。
***************************************************************************
*/
bool MatrixMultiply(int m1, int n1, int m2, int n2, const double M1[], const double M2[], double M3[])
{
    // 首先判断矩阵相乘的大小是否合法
    if (m1 <= 0 || n1 <= 0 || m2 <= 0 || n2 <= 0 || n1 != m2)
    {
        printf("Error dimension in MatrixMultiply!\n");
        return false; // 矩阵大小不合法，无法进行矩阵相乘
    }

    // 执行矩阵相乘运算
    for (int i = 0; i < m1; i++)
    {
        for (int j = 0; j < n2; j++)
        {
            M3[i * n2 + j] = 0; // 初始化结果矩阵元素为0
            for (int k = 0; k < n1; k++)
            {
                M3[i * n2 + j] = M1[i * n1 + k] * M2[k * n2 + j] + M3[i * n2 + j]; // 矩阵相乘
            }
        }
    }

    return true;
}

/*
**********************************************************************************
函数名：矩阵转置
参数：m  M1矩阵的行数
      n  M1矩阵的列数
      M1 原矩阵的数组表示
      MT 转置后矩阵的数组表示
返回值：是否成功进行矩阵转置运算

函数功能：计算矩阵转置，将结果存储在MT数组中。
注：该函数要求矩阵的行数和列数都必须大于0，否则无法进行转置运算，返回false表示失败。
**********************************************************************************
*/
bool MatrixTranspose(int m, int n, const double M1[], double MT[])
{
    // 首先判断矩阵的行数和列数是否合法
    if (m <= 0 || n <= 0)
    {
        printf("Error dimension in MatrixTranspose!\n");
        return false; // 矩阵大小不合法
    }

    // 进行矩阵转置运算
    for (int i = 0; i < m; i++)
    {
        for (int j = 0; j < n; j++)
        {
            MT[j * m + i] = M1[i * n + j]; // 将转置后的元素存储到MT
        }
    }

    return true;
}

/*
**********************************************************************************
函数名：矩阵求逆(采用全选主元高斯-约当法)
参数：n 矩阵的行数、列数
      a 原矩阵的数组表示
      b 求逆后矩阵的数组表示
返回值：是否成功进行矩阵求逆运算

函数功能：计算矩阵求逆（矩阵维度与is和js的数组大小有关），将结果存储在b数组中。
注：返回值为1表示正常，返回值为0表示错误。
**********************************************************************************
*/
int MatrixInv(int n, double a[], double b[])
{
    int i, j, k, l, u, v, is[200], js[200];   
    double d, p;

    if (n <= 0)
    {
        printf("Error dimension in MatrixInv!\n");
        return 0;
    }

    // 将输入矩阵赋值给输出矩阵b，下面对b矩阵求逆，a矩阵不变
    for (i = 0; i < n; i++)
    {
        for (j = 0; j < n; j++)
        {
            b[i * n + j] = a[i * n + j];
        }
    }

    for (k = 0; k < n; k++)
    {
        d = 0.0;
        for (i = k; i < n; i++)   // 查找右下角方阵中主元素的位置 
        {
            for (j = k; j < n; j++)
            {
                l = n * i + j;
                p = fabs(b[l]);
                if (p > d)
                {
                    d = p;
                    is[k] = i;
                    js[k] = j;
                }
            }
        }

        if (d < DBL_EPSILON)   // 主元素接近于0，矩阵不可逆
        {
            printf("Divided by 0 in MatrixInv!\n");
            return 0;
        }

        if (is[k] != k)  // 对主元素所在的行与右下角方阵的首行进行调换
        {
            for (j = 0; j < n; j++)
            {
                u = k * n + j;
                v = is[k] * n + j;
                p = b[u];
                b[u] = b[v];
                b[v] = p;
            }
        }

        if (js[k] != k)  // 对主元素所在的列与右下角方阵的首列进行调换
        {
            for (i = 0; i < n; i++)
            {
                u = i * n + k;
                v = i * n + js[k];
                p = b[u];
                b[u] = b[v];
                b[v] = p;
            }
        }

        l = k * n + k;
        b[l] = 1.0 / b[l];  // 初等行变换
        for (j = 0; j < n; j++)
        {
            if (j != k)
            {
                u = k * n + j;
                b[u] = b[u] * b[l];
            }
        }
        for (i = 0; i < n; i++)
        {
            if (i != k)
            {
                for (j = 0; j < n; j++)
                {
                    if (j != k)
                    {
                        u = i * n + j;
                        b[u] = b[u] - b[i * n + k] * b[k * n + j];
                    }
                }
            }
        }
        for (i = 0; i < n; i++)
        {
            if (i != k)
            {
                u = i * n + k;
                b[u] = -b[u] * b[l];
            }
        }
    }

    for (k = n - 1; k >= 0; k--)  // 将上面的行列调换重新恢复
    {
        if (js[k] != k)
        {
            for (j = 0; j < n; j++)
            {
                u = k * n + j;
                v = js[k] * n + j;
                p = b[u];
                b[u] = b[v];
                b[v] = p;
            }
        }
        if (is[k] != k)
        {
            for (i = 0; i < n; i++)
            {
                u = i * n + k;
                v = is[k] + i * n;
                p = b[u];
                b[u] = b[v];
                b[v] = p;
            }
        }
    }

    return (1);
}


/*
****************************************
函数名：矩阵重构函数1
参数：m  M矩阵的行数
      n  M矩阵的列数
      m1 待删除行序号
      n1 待删除列序号
      M  矩阵的数组表示
函数功能：删除矩阵的指定一行和一列。
****************************************
*/
void deleteRowAndColumn(int m, int n, int m1, int n1, double M[]) 
{
    m1 = m1 - 1;
    n1 = n1 - 1;
    // 矩阵第m1行元素置零
    for (int j = 0; j < n; j++) {
        M[m1 * n + j] = 0;
    }
    // 矩阵第n1列元素置零
    for (int i = 0; i < m; i++) {
        M[i * n + n1] = 0;
    }
    int count = 0; // 初始化一个计数器，用来记录非零元素的个数
    int length = m * n;
   // 遍历数组，把非零元素挪到数组靠前位置
    for (int i = 0; i < length; i++)
    {
        if (M[i] != 0)
        {
            M[count] = M[i]; // 将非零元素移到数组的前面
            count++;
        }
    }
    // 将剩余的位置全部填充为0
    while (count < length)
    {
        M[count++] = 0;
    }
}

/*
********************************
函数名：矩阵重构函数2
参数：rows  列向量的行数
      rowToDelete 待删除行序号
      vector  列向量的数组表示
函数功能：删除矩阵的指定一行。
********************************
*/
void deleteRow(int rows,int rowToDelete ,double vector[]) 
{
    // 将待删除的行的所有元素置零
    for (int i = 0; i < rows; i++)
    {
        if (i == (rowToDelete-1)) 
        {
            vector[i] = 0;
            break;
        }
    }
    int count = 0; // 初始化一个计数器，用来记录非零元素的个数
    int length = rows;
    // 遍历数组，把非零元素挪到数组靠前位置
    for (int i = 0; i < length; i++)
    {
        if (vector[i] != 0)
        {
            vector[count] = vector[i]; // 将非零元素移到数组的前面
            count++;
        }
    }
    // 将剩余的位置全部填充为0
    while (count < length)
    {
        vector[count++] = 0;
    }
}



void restructureMatrix(int a, int b, int m, int n, double A[]) 
{
    double* temp = (double*)malloc(a * b * sizeof(double)); // 创建一个临时数组来存储重构后的数据
    if (temp == NULL) {
        fprintf(stderr, "Memory allocation failed\n");
        return;
    }

    for (int i = 0; i < a; ++i) {
        for (int j = 0; j < b; ++j) {
            temp[i * b + j] = A[i * n + j]; // 从A中拷贝实际矩阵的数据到temp数组
        }
    }

    for (int i = 0; i < a * b; ++i) {
        A[i] = temp[i]; // 将重构后的数据拷贝回A数组
    }

    // 重置A数组剩余的部分为0
    for (int i = a * b; i < m * n; ++i) {
        A[i] = 0.0;
    }

    free(temp); // 释放临时数组
}
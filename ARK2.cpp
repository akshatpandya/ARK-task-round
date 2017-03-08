#include <stdio.h>
#include <iostream>
#include <math.h>
#include <utility>
#include <stack>
#include <string>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

typedef pair<int, int> Pair;
typedef pair<double, pair<int, int> > Pair2;
 
//structure for storing the values in my graph
struct node
{
    int from_x, from_y;
    double f, g, h;
};

//to check if the corresponding point lies in the matrix
bool isValid(int row, int col, int m, int n)
{
    return (row >= 0) && (row < m) && (col >= 0) && (col < n);
}


//this the main function that calculates the path
void astar()
{
	int i, j, m, n, check_value,Xstart=0, Ystart=0, Xend=0, Yend=0, bot1X=0, bot1Y=0, bot2X=0, bot2Y=0;
	
	//taking input for the no. of rows and no. of columns
	cin>>m;
	cin>>n;	
	
	//creating a frid that willbe my map henceforth, obstaclesare marked 1 and open spaces 0 
    int **grid = (int**)malloc(m * sizeof(int*));
    for(i=0;i<m;i++)
		grid[i]= (int*)malloc(n * sizeof(int));
		
	for(i=0; i<m; i++)
    {
		for(j=0; j<n; j++)
		{
			cin>>check_value;
			if(check_value==1)
				grid[i][j] = 1;
			else if(check_value==0)
				grid[i][j] = 0;
			else if(check_value==5)
			{
				bot1X = i;
				bot1Y = j;
			}
			else if(check_value==6)
			{
				bot2X = i;
				bot2Y = j;
			}	
			else if(check_value==10)
			{
				Xend = i;
				Yend = j;
			}	
		}
	}
	
	Pair bot1 = make_pair(bot1X, bot1Y);
	Pair bot2 = make_pair(bot2X, bot2Y);
	
	//in this section we calculate the position till where the 2 bots must come indivisually, for this we first calculate the 
	//midpoint of the 2 bots and check if the it is an obstacle or an open space, if an open space my two bots must go there
	//if not, then it searches for the point that is open and almostin the mid of the 2 bots and closer to the destination  
	
	int midX, midY, togoX, togoY;
	midX = (bot1X + bot2X)/2;
	midY = (bot1Y + bot2Y)/2;
	
	cout<<"midX = "<<midX<<" midY  = "<<midY<<endl;
	cout<<grid[midX][midY]<<endl;
	
	//if the midpoint is not an obstacle
	if(grid[midX][midY]==0)
	{
		togoX = midX;
		togoY = midY;
		Xstart = midX;
		Ystart = midX;
	} 
	//if the midpoint is an obstacle
	else if(grid[midX][midY]==1)
	{
		int flag = 0, a,b;
		//this marked matrix stores the visited nodes
		bool **marked = (bool**)malloc(m * sizeof(bool*));
		for(a=0;a<m;a++)
			marked[a]= (bool*)malloc(n * sizeof(bool));
		for(a=0; a<m; a++)
		{
			for(b=0; b<n; b++)
			{
				marked[a][b] = false;
			}
		}
		//this priority_list stores the yet not marked nodes in ascending order of their distances from the destination
		set<Pair2>priority_list;
		priority_list.insert(make_pair (0.0, make_pair(midX, midY)));
		
		//my search for the point starts
		
		while(!priority_list.empty())
		{
			int i,j,k,l;
			Pair2 temp = *priority_list.begin();
			priority_list.erase(priority_list.begin());
 
			i = temp.second.first;
			j = temp.second.second;
			marked[i][j] = true;
			
			for(k=-1;k<=1;k++)
			{
				for(l=-1;l<=1;l++)
				{
					if(grid[i+k][j+l]==0)
					{
						togoX = i+k;
						togoY = j+l;
						Xstart = i+k;
						Ystart = j+l;
						flag=1;
						break;
					}
					else
					{
						double distance = sqrt((i+k-Xend)*(i+k-Xend) + (j+l-Yend)*(j+l-Yend));
						//cout<<"Distance "<<distance<<endl;
						
						priority_list.insert(make_pair(distance,make_pair(i+k, j+l)));
					}
				}
				if(flag==1)
					break;
			}
			if(flag==1)
				break;
		}
	}
	
	//togoX and togoY are the x and y co-ordinates of the common point where the 2 bots reach
	cout<<"togoX = "<<togoX<<" togoY = "<<togoY<<endl;
	cout<<"startX = "<<Xstart<<" Ystart = "<<Ystart<<endl;

//for bot1 and bot2, computing the smallest distance to the common point, by A*

	Pair togo_dest = make_pair(togoX, togoY);
	Pair start[2]  = {make_pair(bot1X, bot1Y), make_pair(bot2X, bot2Y)};
	
	for(i=0;i<2;i++)
	{
		int g,h;
		bool **closedList = (bool**)malloc(m * sizeof(bool*));
		for(g=0;g<m;g++)
			closedList[g]= (bool*)malloc(n * sizeof(bool));
		for(g=0; g<m; g++)
		{
			for(h=0; h<n;h++)
			{
				closedList[g][h] = false;
			}
		}
		
		node **graph = (node**)malloc(m * sizeof(node*));
		for(g=0;g<m;g++)
			graph[g]= (node*)malloc(n * sizeof(node));
		
	 
		for (g=0; g<m; g++)
		{
			for (h=0; h<n; h++)
			{
				graph[g][h].f = FLT_MAX;
				graph[g][h].g = FLT_MAX;
				graph[g][h].h = FLT_MAX;
				graph[g][h].from_x = -1;
				graph[g][h].from_y = -1;
			}
		}

		g = start[i].first, h = start[i].second;
		graph[g][h].f = 0.0;
		graph[g][h].g = 0.0;
		graph[g][h].h = 0.0;
		graph[g][h].from_x = g;
		graph[g][h].from_y = h;

		set<Pair2> openList;
		openList.insert(make_pair (0.0, make_pair (g, h)));
		bool foundDest = false;
		cout<<"g = "<<g<<" h = "<<h<<endl;
		while (!openList.empty())
		{
			Pair2 p = *openList.begin();
	 
			openList.erase(openList.begin());
	 
			g = p.second.first;
			h = p.second.second;
			closedList[g][h] = true;
	   
			double gNew, hNew, fNew;
			int k,l;
			
			for(k=-1;k<=1;k++)
			{
				for(l=-1;l<=1;l++)
				{
					 if (isValid(g+k, h+l, m, n) == true && !(k==0 && l==0))
					 {
						if ((g+k) == togo_dest.first && (h+l) == togo_dest.second)
						{
							graph[g+k][h+l].from_x = g;
							graph[g+k][h+l].from_y = h;
							int row = togo_dest.first;
							int col = togo_dest.second;
							while (!(graph[row][col].from_x == row && graph[row][col].from_y == col))
							{
								cout<<"("<<row<<","<<col<<")";
								int temp_row = graph[row][col].from_x;
								int temp_col = graph[row][col].from_y;
								row = temp_row;
								col = temp_col;
							}
							foundDest = true;
							cout<<"("<<row<<","<<col<<")";
							cout<<endl;
							cout<<endl;
							cout<<endl;
							break;
						}

						else if (closedList[g+k][h+l] == false && grid[g+k][h+l] == 0)
						{
							gNew = graph[g][h].g + ((k==0||l==0)?1:1.414);
							hNew = (double)sqrt (((g+k)-togo_dest.first)*((g+k)-togo_dest.first) + ((h+l)-togo_dest.second)*((h+l)-togo_dest.second));
							fNew = gNew + hNew;
			 
							if (graph[g+k][h+l].f == FLT_MAX || graph[g+k][h+l].f > fNew)
							{
								openList.insert( make_pair(fNew,make_pair(g+k, h+l)));
								graph[g+k][h+l].f = fNew;
								graph[g+k][h+l].g = gNew;
								graph[g+k][h+l].h = hNew;
								graph[g+k][h+l].from_x = g;
								graph[g+k][h+l].from_y = h;
							}
						}
					}
				}
				if(foundDest==true)
					break;
			} 
			if(foundDest==true)
					break;  
		}
	}


	//now, computing the common path for the bots from the common point to the destination by A*
	Pair src = make_pair(Xstart, Ystart);
    Pair dest = make_pair(Xend, Yend);
	

    bool **closedList = (bool**)malloc(m * sizeof(bool*));
	for(i=0;i<m;i++)
		closedList[i]= (bool*)malloc(n * sizeof(bool));
    for(i=0; i<m; i++)
    {
		for(j=0; j<n; j++)
		{
			closedList[i][j] = false;
		}
	}
    
    
    
	node **graph = (node**)malloc(m * sizeof(node*));
	for(i=0;i<m;i++)
		graph[i]= (node*)malloc(n * sizeof(node));
    
 
    for (i=0; i<m; i++)
    {
        for (j=0; j<n; j++)
        {
            graph[i][j].f = FLT_MAX;
            graph[i][j].g = FLT_MAX;
            graph[i][j].h = FLT_MAX;
            graph[i][j].from_x = -1;
            graph[i][j].from_y = -1;
        }
    }

    i = src.first, j = src.second;
    graph[i][j].f = 0.0;
    graph[i][j].g = 0.0;
    graph[i][j].h = 0.0;
    graph[i][j].from_x = i;
    graph[i][j].from_y = j;

    set<Pair2> openList;
 
    openList.insert(make_pair (0.0, make_pair (i, j)));
 

    bool foundDest = false;
 
    while (!openList.empty())
    {
        Pair2 p = *openList.begin();
 
        openList.erase(openList.begin());
 
        i = p.second.first;
        j = p.second.second;
        closedList[i][j] = true;
   
        double gNew, hNew, fNew;
 
		int k,l;
		
		for(k=-1;k<=1;k++)
		{
			for(l=-1;l<=1;l++)
			{
				 if (isValid(i+k, j+l, m, n) == true && !(k==0 && l==0))
				 {
					 
					if ((i+k) == dest.first && (j+l) == dest.second)
					{
						graph[i+k][j+l].from_x = i;
						graph[i+k][j+l].from_y = j;
						int row = dest.first;
						int col = dest.second;
						while (!(graph[row][col].from_x == row && graph[row][col].from_y == col ))
						{
							cout<<"("<<row<<","<<col<<")";
							int temp_row = graph[row][col].from_x;
							int temp_col = graph[row][col].from_y;
							row = temp_row;
							col = temp_col;
						}
						cout<<"("<<row<<","<<col<<")";
						foundDest = true;
						cout<<endl;
						cout<<endl;
						cout<<endl;
						return;
					}

					else if (closedList[i+k][j+l] == false && grid[i+k][j+l] == 0)
					{
						gNew = graph[i][j].g + ((k==0||l==0)?1:1.414);
						hNew = (double)sqrt (((i+k)-dest.first)*((i+k)-dest.first) + ((j+l)-dest.second)*((j+l)-dest.second));
						fNew = gNew + hNew;
		 
						if (graph[i+k][j+l].f == FLT_MAX || graph[i+k][j+l].f > fNew)
						{
							openList.insert( make_pair(fNew,make_pair(i+k, j+l)));
							graph[i+k][j+l].f = fNew;
							graph[i+k][j+l].g = gNew;
							graph[i+k][j+l].h = hNew;
							graph[i+k][j+l].from_x = i;
							graph[i+k][j+l].from_y = j;
						}
					}
				}
			}
		}   
    }
    return;
}
 
 //we redirect the standard input and output to the files input2.txt and output2.txt
int main(int argc, char *argv[])
{
	int i;
		astar();
    return(0);
}

#ifndef READ_MYSQL_CORE_H
#define READ_MYSQL_CORE_H

#include <stdlib.h>
#include <mysql.h>
#include <string>
#include <string.h>
#include <iostream>
#include<vector>
#include "datatype.h"
#include <pqxx/pqxx>

using namespace std;
using namespace pqxx;

class Read_Mysql_Core
{
public:
    Read_Mysql_Core(DBAddr &db_addr);                 //构造函数
    void exec();                         //激活回调函数
    vector<RoadAttribute> road_atr_vec; //RoadAttribute集合
    vector<TaskAttribute> task_atr_vec; //TaskAttribute集合
    bool connect_is_ok = true;
    bool query_is_ok = true;

    void MysqlInit();
    void Read_Road_Attri();
    void Read_Task_Attri();
    void Free_Mysql();
	void Read_Pqdata();

private:
    struct RoadAttribute road_atr;      //从数据库读取的RoadAttribute
    struct TaskAttribute task_atr;      //从数据库读取的TaskAttribute
	//mysql 配置
	char user[20];
	char pswd[20];
	char host[20];    //mysql服务器地址
	char dbname[20];   //要连接的数据库名称
	unsigned int port ;   

	//postgre配置
	string pq_dbname;
	string pq_user;
	string pq_password;
	string pq_hostaddr;
	string pq_port;

	MYSQL mysql1;
	MYSQL_RES *mysql_result;
	MYSQL_ROW sql_row;
	MYSQL_FIELD *fields;
	int res;

};



Read_Mysql_Core::Read_Mysql_Core(DBAddr &db_addr)
{
	strcpy(user,db_addr.user);
	strcpy(pswd,db_addr.pswd);
	strcpy(host,db_addr.host);    
	strcpy(dbname,db_addr.dbname);  
	port = db_addr.port; 

	pq_dbname = db_addr.dbname;
	pq_user = db_addr.user;
	pq_password = db_addr.pswd;
	pq_hostaddr = db_addr.host;
	pq_port = to_string(db_addr.port); 
}


void Read_Mysql_Core::Read_Pqdata()
{
  const char* sql;
  try{

    connection conn("dbname="+pq_dbname+" user="+pq_user+" password="+pq_password+" hostaddr="+pq_hostaddr+" port="+pq_port);
	if(conn.is_open()){
		connect_is_ok = true;
      cout<<"connect to postgresql successfully. dbname="<<conn.dbname()<<endl;
    }else{
		connect_is_ok = false;
      cout<<"connect to postgresql fail."<<endl;
      return ;
    }
    // work tnx(conn);
    // sql = "insert into xx_user(id,name,mobile,birth) values (1,'aaa','15011186301',current_date),(2,'bbb','15910909520',current_date)";
    // tnx.exec(sql);
    // tnx.commit();
    nontransaction ntx(conn);
    sql = "select * from attri_tb";
    result r(ntx.exec(sql));
    for(result::const_iterator c=r.begin(); c!=r.end(); ++c){
        road_atr.id_map = c[0].as<int>();
        road_atr.velocity = c[1].as<double>();
        road_atr.road_width = c[2].as<double>();
        road_atr.aeb_front = c[3].as<double>();
	    road_atr.aeb_back = c[4].as<double>();
        road_atr.aeb_left = c[5].as<double>();
	    road_atr.aeb_right = c[6].as<double>();        
		road_atr.start_s = c[7].as<double>();
		road_atr.end_s = c[8].as<double>();
		road_atr_vec.push_back(road_atr);
	//cout<<"_______________________________________"<<endl;
    } 

    sql = "select * from task_tb";
    result rr(ntx.exec(sql));
    for(result::const_iterator c=rr.begin(); c!=rr.end(); ++c){
        task_atr.id_task = c[0].as<int>();
		task_atr.task_type = c[1].as<int>();
		task_atr.start_s = c[2].as<double>();
		task_atr.end_s = c[3].as<double>();
		task_atr.task_info = c[4].as<double>();
		task_atr.notes = const_cast<char*>( c[5].as<string>().c_str() );    
		task_atr_vec.push_back(task_atr);   
	//cout<<"_______________________________________"<<endl;
    } 

    conn.disconnect();
  }catch(const std::exception &e){
	cout<<"postgre connection error!!!!!!!"<<endl;
	connect_is_ok = false;
    cerr<<e.what()<<endl;
    return ;
  }

}



void Read_Mysql_Core::MysqlInit()
{
    mysql_init(&mysql1);
}




void Read_Mysql_Core::Read_Road_Attri()
{
    if (mysql_real_connect(&mysql1, host, user, pswd, dbname, port, NULL, 0))
	{
        connect_is_ok = true;
		mysql_query(&mysql1, "SET NAMES GBK");                   //设置编码格式
		res = mysql_query(&mysql1, "select * from attri_tb");    //查询
		if (!res)
		{
            query_is_ok = true;
			mysql_result = mysql_store_result(&mysql1);
			if (mysql_result)
			{
				unsigned int num_column, num_row;	        //记录表格大小

				num_column = mysql_num_fields(mysql_result);    //返回结果集中字段（列）的数量
				num_row = mysql_num_rows(mysql_result);         //返回结果集中行的数量

        //mysql_fetch_field() 函数从结果集中取得列信息并作为对象返回。
				 fields = mysql_fetch_field(mysql_result);
       
        //mysql_fetch_row( ) 返回结果集中的下一行，如果没有更多行则返回 FALSE。可依次获得每一行的内容
				while (sql_row = mysql_fetch_row(mysql_result))//按行输出数据
				{
					for (int i = 0; i < num_column; ++i)
					{
//						cout << sql_row[i] << "\t";  
                        switch (i)
						{
							case 0:
								road_atr.id_map = atoi(sql_row[i]);
								break;
							case 1:
								road_atr.velocity = atof(sql_row[i]);
								break;
							case 2:
								road_atr.road_width = atof(sql_row[i]);
								break;
							case 3:
								road_atr.aeb_front = atof(sql_row[i]);
								break;
							case 4:
								road_atr.aeb_back = atof(sql_row[i]);
								break;
							case 5:
								road_atr.aeb_left = atof(sql_row[i]);
								break;
							case 6:
								road_atr.aeb_right = atof(sql_row[i]);
								break;
							case 7:
								road_atr.detect_front = atof(sql_row[i]);
								break;
							case 8:
								road_atr.detect_back = atof(sql_row[i]);
								break;
							case 9:
								road_atr.detect_left = atof(sql_row[i]);
								break;
							case 10:
								road_atr.detect_right = atof(sql_row[i]);
								break;
							case 11:
								road_atr.start_s = atof(sql_row[i]);
								break;
							case 12:
								road_atr.end_s = atof(sql_row[i]);
								break;
							
							default:
								break;
						} 

					}
                    road_atr_vec.push_back(road_atr);

				}

			}
		}
		   else
		   {
			query_is_ok = false;
		    }

	}
	else
	{
		connect_is_ok = false;
	}
}





void Read_Mysql_Core::Read_Task_Attri()
{
    //if (mysql_real_connect(&mysql1, host, user, pswd, dbname, port, NULL, 0))    //Read_Road_Attri已connect DB
	//{
        connect_is_ok = true;
		mysql_query(&mysql1, "SET NAMES GBK");                   //设置编码格式
		res = mysql_query(&mysql1, "select * from task_tb");    //查询
		if (!res)
		{
            query_is_ok = true;
			mysql_result = mysql_store_result(&mysql1);
			if (mysql_result)
			{
				unsigned int num_column, num_row;	        
				num_column = mysql_num_fields(mysql_result);    
				num_row = mysql_num_rows(mysql_result);         

				 fields = mysql_fetch_field(mysql_result);
    
				while (sql_row = mysql_fetch_row(mysql_result))
				{
					for (int i = 0; i < num_column; ++i)
					{ 
                        switch (i)
						{
							case 0:
								task_atr.id_task = atoi(sql_row[i]);
								break;
							case 1:
								task_atr.id_map = atoi(sql_row[i]);
								break;
							case 2:
								task_atr.task_type = atoi(sql_row[i]);
								break;
							case 3:
								task_atr.start_s = atof(sql_row[i]);
								break;
							case 4:
								task_atr.end_s = atof(sql_row[i]);
								break;
							case 5:
								task_atr.task_info = atoi(sql_row[i]);
								break;
							case 6:
								task_atr.notes = sql_row[i];
								break;
						   default:
								break;
						} 

					}
                    task_atr_vec.push_back(task_atr);

				}

			}
		}
	  else
		{
		query_is_ok = false;
		}

	//}
	// else
	// {
	// 	connect_is_ok = false;
	// }
}






void Read_Mysql_Core::exec()
{



}



void Read_Mysql_Core::Free_Mysql()
{
    if (mysql_result != NULL)
	mysql_free_result(mysql_result);

	mysql_close(&mysql1);
}




#endif // READ_MYSQL_CORE_H

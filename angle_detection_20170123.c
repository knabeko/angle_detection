#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define MAX_SMOOTHING_DATA_NUMBER 3000//2000//平滑化するデータの数
#define SMOOTHING_TIMES 100//平滑化する回数

#define MAX_SAMPLE_DATA_NUMBER 3000//2000,ここの値はMAX_SMOOTHING_DATA_NUMBER以下の値でなければならない
#define TIME_DELTA 0.003//0.001、極値を取ったタイミングからどれだけの時間の範囲のモーターのデータを参照するか

#define AZIMUTH_SAMPLE_NUMBER 10//データとして採用する方位角のデータ数
#define ELEVATION_SAMPLE_NUMBER 10//データとして採用する仰角のデータ数

#define PI 3.141592653589//円周率の定義

#define FACTOR 1//仮の定数
#define UNIXTIME 0//1479435273//時刻の基準、この値は仮のもの

void position(char *sen_mag_file, char *mag_file, char *norm_file, char *motor_file);
void azimuth(char *sen_mag_file, char *mag_file, char *norm_file, char *motor_file);
void elevation(char *sen_mag_file, char *mag_file, char *norm_file, char *motor_file);
void orientation(char *mag_file, char *sen_acc_file, char *mag_file_2);

void file_read(FILE **file, char *argv);//ファイルを読み込み可能状態にする関数
void file_write(FILE **file, char *argv);//ファイルを書き込み可能状態にする関数

void smoothing(FILE *file, char *argv);//データを平滑化する関数、この関数は未使用
void smoothing_xyz(FILE *file, char *argv);//x,y,z成分を同時に平滑化する関数

double radian_to_degree(double angle);//角度を弧度法から度数法に変換する関数
double get_average(double *data, int data_number);//データの平均値を求める関数
double get_max(double *data, int data_number);//データの最大値を求める関数
double get_min(double *data, int data_number);//データの最小値を求める関数
double get_deviation(double *data, int data_number);//データの分散（標準偏差を求める処理）
double get_median(double *data, int data_number);//データの最大値、最小値、平均を求め、それらから関数の極値を求めて中央値を求める関数
void get_extreme_value(double *data, int data_number, double *ext_max_data, int *ext_max_data_number, double *ext_min_data, int *ext_min_data_number);//極値を求める関数
void get_extreme_norm_and_unixtime(double *data, int data_number, double *unixtime, double *ext_max_time, int *ext_max_time_number, double *ext_min_time, int *ext_min_time_number, double *ext_max_data, double *ext_min_data);//ノルムの極大値と極小値を求め、さらにそれぞれの値をとった時の時間を求める変数
void rotate_x(double *data_x, double *data_y, double *data_z, int data_number, double angle);//x軸を中心にデータを回転させる関数
void rotate_y(double *data_x, double *data_y, double *data_z, int data_number, double angle);//y軸を中心にデータを回転させる関数
void rotate_z(double *data_x, double *data_y, double *data_z, int data_number, double angle);//z軸を中心にデータを回転させる関数
void test();//なんかのテスト用



//以下、データからノルムを求める処理における変数
FILE *fp_sen_mag;//センサーの磁気データ
FILE *fp_mag;//平滑化したのち、中央値を0に近づけたセンサーの磁気データファイル
FILE *fp_norm;//ノルムのデータファイル

double mag_x[MAX_SAMPLE_DATA_NUMBER];
double mag_y[MAX_SAMPLE_DATA_NUMBER];
double mag_z[MAX_SAMPLE_DATA_NUMBER];
int mag_data_number = 0;

double median_x = 0;
double median_y = 0;
double median_z = 0;

double mag_norm = 0;
double mag_norm_data[MAX_SAMPLE_DATA_NUMBER];
int mag_norm_data_number = 0;
double norm_average = 0;
double unixtime[MAX_SAMPLE_DATA_NUMBER];//ローカル変数と名前がかぶっている

double ave_x = 0;
double ave_y = 0;
double ave_z = 0;

double x_max = 0;
double y_max = 0;
double z_max = 0;

double x_min = 0;
double y_min = 0;
double z_min = 0;

int ret;//ファイル読み込み用変数
double f1, f2, f3, f4, f5;//ファイル読み込み用変数

//以下、スマホの磁場マーカからの方位角を検出する処理の変数
FILE *fp_motor;//モーターの角度データ

double ext_max_data[MAX_SAMPLE_DATA_NUMBER];//ノルムの極大値の配列
double ext_min_data[MAX_SAMPLE_DATA_NUMBER];//ノルムの極小値の配列
double ext_max_time[MAX_SAMPLE_DATA_NUMBER];//ノルムが極大値を取った時間の配列
int ext_max_time_number = 0;//ノルムが極大値を取った時間の回数
double ext_min_time[MAX_SAMPLE_DATA_NUMBER];//ノルムが極小を取った時間の配列
int ext_min_time_number = 0;//ノルムが極小値を取った時間の回数

double motor_angle = 0;
double motor_time = 0;

double angle_sample[MAX_SAMPLE_DATA_NUMBER];
double mag_sample[MAX_SAMPLE_DATA_NUMBER];
double time_sample[MAX_SAMPLE_DATA_NUMBER];

int number_of_cnt_ext_max_time = 0;

double azimuth_1_sample[MAX_SAMPLE_DATA_NUMBER];
int cnt_azimuth_1_sample = 0;
double azimuth_2_sample[MAX_SAMPLE_DATA_NUMBER];
int cnt_azimuth_2_sample = 0;

double azimuth_1_average = 0;
double azimuth_2_average = 0;
double azimuth_1_deviation = 0;
double azimuth_2_deviation = 0;

double azimuth_1_norm = 0;//方位角１が検出された時のノルム
double azimuth_2_norm = 0;//方位角２が検出された時のノルム



int main(int argc, char *argv[]){

  //磁気センサーの値からスマホの位置を求める処理
  position(argv[1], argv[2], argv[3], argv[4]);//磁気センサーのデータファイルとモーターの角度のデータファイルを渡す

  //加速度センサからスマホの姿勢を求める処理
  orientation(argv[2], argv[5], argv[6]);//磁気センサーのデータファイルと加速度センサーのデータファイルを渡す

  return;
}



void position(char *sen_mag_file, char *mag_file, char *norm_file, char *motor_file){
  azimuth(sen_mag_file, mag_file, norm_file, motor_file);//方位角を求める処理
  elevation(sen_mag_file, mag_file, norm_file, motor_file);//仰角を求める処理
  return;
}

void azimuth(char *sen_mag_file, char *mag_file, char *norm_file, char *motor_file){
  int cnt_ext_max_time = 0;
  int cnt_angle_sample = 0;

  file_read(&fp_sen_mag, sen_mag_file);
  file_write(&fp_mag, mag_file);

  //fp_sen_magのデータをfp_magにコピー
  while( ( ret = fscanf( fp_sen_mag, "%lf, %lf, %lf, %lf, %lf\n", &f1, &f2, &f3, &f4, &f5 ) ) != EOF ){
    fprintf(fp_mag, "%lf,%lf,%lf,%lf\n",f1, f3, f4, f5);
  }

  fclose(fp_mag);
  fclose(fp_sen_mag);

  //磁場の各成分を同時に平滑化する処理
  smoothing_xyz(fp_mag, mag_file);

  //以下、ノルムを求める処理
  //磁場のデータを配列に格納する
  file_read(&fp_mag, mag_file);
  while(fscanf(fp_mag,"%lf,%lf,%lf,%lf\n",&f1,&f2,&f3,&f4)!=EOF) {
    if(mag_data_number < MAX_SAMPLE_DATA_NUMBER){
      mag_x[mag_data_number] = f2;
      mag_y[mag_data_number] = f3;
      mag_z[mag_data_number] = f4;
      mag_data_number ++;
    }
  }
  fclose(fp_mag);

  //各成分の平均を求める処理、確認用で処理には使っていない
  ave_x = get_average(mag_x, mag_data_number);
  ave_y = get_average(mag_y, mag_data_number);
  ave_z = get_average(mag_z, mag_data_number);
  printf("average x:%lf, y:%lf, z:%lf\n", ave_x,ave_y,ave_z);

  //各磁場データの中央値を求め、中央値が0になるように修正する処理
  median_x = get_median(mag_x, mag_data_number);
  median_y = get_median(mag_y, mag_data_number);
  median_z = get_median(mag_z, mag_data_number);
  printf("median x:%lf y:%lf z:%lf\n\n", median_x, median_y, median_z);

  //fp_magをfp_normにコピー
  file_read(&fp_mag, mag_file);
  file_write(&fp_norm, norm_file);

  while( ( ret = fscanf( fp_mag, "%lf,%lf,%lf,%lf\n", &f1, &f2, &f3, &f4 ) ) != EOF ){
    fprintf(fp_norm, "%lf,%lf,%lf,%lf\n", f1, f2, f3, f4 );
  }

  fclose(fp_norm);
  fclose(fp_mag);

  //修正した磁場のデータをfp2に書き込む
  file_read(&fp_norm, norm_file);
  file_write(&fp_mag, mag_file);

  while( ( ret = fscanf( fp_norm, "%lf, %lf, %lf, %lf\n", &f1, &f2, &f3, &f4 ) ) != EOF ){
    fprintf(fp_mag, "%lf,%lf,%lf,%lf\n",f1 - UNIXTIME, f2 - median_x, f3 - median_y, f4 - median_z);
    //fprintf(fp2, "%lf,%lf,%lf,%lf\n",f1, f2 - median_x, f3 - median_y, f4 - f4);
  }

  fclose(fp_mag);
  fclose(fp_norm);

  //修正した磁場データを配列に格納する
  file_read(&fp_mag, mag_file);
  mag_data_number = 0;
  while(fscanf(fp_mag,"%lf,%lf,%lf,%lf\n",&f1,&f2,&f3,&f4)!=EOF) {
    if(mag_data_number < MAX_SAMPLE_DATA_NUMBER){
      mag_x[mag_data_number] = f2;
      mag_y[mag_data_number] = f3;
      mag_z[mag_data_number] = f4;
      mag_data_number ++;
    }
  }
  fclose(fp_mag);

  //ノルムの計算、fp2から計算したデータをfp3に書き込む
  file_read(&fp_mag, mag_file);
  file_write(&fp_norm, norm_file);

  while( ( ret = fscanf( fp_mag, "%lf, %lf, %lf, %lf\n", &f1, &f2, &f3, &f4 ) ) != EOF ){
    mag_norm = pow(f2,2) + pow(f3,2) + pow(f4,2);
    mag_norm = sqrt(mag_norm);
    fprintf(fp_norm, "%lf,%lf\n", f1 - UNIXTIME, mag_norm);
  }

  fclose(fp_norm);
  fclose(fp_mag);

  //ノルムのデータを配列に格納
  file_read(&fp_norm, norm_file);

  while( ( ret = fscanf( fp_norm, "%lf, %lf\n", &f1, &f2 ) ) != EOF ){
    unixtime[mag_norm_data_number] = f1;
    mag_norm_data[mag_norm_data_number] = f2;
    mag_norm_data_number++;
  }

  fclose(fp_norm);

  norm_average = get_average(mag_norm_data, mag_norm_data_number);
  printf("norm_average:%lf\n\n", norm_average);



  //ノルムのファイルにモーターの相対時間を書き込む（仮の処理、あとで消去する予定）
  //file_read(&fp_motor, motor_file);
  //file_write(&fp_norm, norm_file);
  //while( ( ret = fscanf( fp_motor, "%lf, %lf\n", &f1, &f2 ) ) != EOF ){
  //  fprintf(fp_norm, "%lf,%lf\n", f1 - UNIXTIME, f2);
  //}
  //fclose(fp_norm);
  //fclose(fp_motor);



  //ここまででやったこと：
  //磁場のデータを平滑化＋中央値あわせ、それらのデータをmag_x[mag_data_number]、mag_y[mag_data_number]、mag_z[mag_data_number]に格納
  //それらからノルムのデータを求め、それらのデータをunixtime[mag_norm_data_number]、mag_norm_data[mag_norm_data_number]に格納

  //以下、スマホの磁場マーカからの方位角を検出する処理（改良の必要があるかもしれない）
  get_extreme_norm_and_unixtime(mag_norm_data, mag_norm_data_number, unixtime, ext_max_time, &ext_max_time_number, ext_min_time, &ext_min_time_number, ext_max_data, ext_min_data);

  //ノルムが極大値を取るunixtimeにおけるモータの角度データを取得する処理
  file_read(&fp_motor, motor_file);

  while(fscanf(fp_motor,"%lf,%lf\n",&motor_time,&motor_angle)!=EOF) {
    for(cnt_ext_max_time=0; cnt_ext_max_time<=ext_max_time_number; cnt_ext_max_time++){
      if(ext_max_time[cnt_ext_max_time] - TIME_DELTA <= motor_time && motor_time <= ext_max_time[cnt_ext_max_time] + TIME_DELTA){
        angle_sample[cnt_angle_sample] = motor_angle;
        mag_sample[cnt_angle_sample] = ext_max_data[cnt_ext_max_time];
        time_sample[cnt_angle_sample] = motor_time;
        cnt_angle_sample ++;
      }
    }
  }

  number_of_cnt_ext_max_time = cnt_angle_sample - 1;
  //printf("%d\n", number_of_cnt_ext_max_time);

  //angle_sampleの１つ目の値から２つの角度を分類する基準を決める（この辺り改良が必要かもしれない）
  double azimuth_threshold = 0;
  if(0<=angle_sample[0] && angle_sample[0]<90){
    azimuth_threshold = angle_sample[0] + 90;
  }else if(90<=angle_sample[0] && angle_sample[0]<180){
    azimuth_threshold = angle_sample[0] - 90;
  }else if(180<=angle_sample[0] && angle_sample[0]<270){
    azimuth_threshold = angle_sample[0] - 90;
  }else if(270<=angle_sample[0] && angle_sample[0]<360){
    azimuth_threshold = angle_sample[0] - 270;
  }
  //確認用の表示
  printf("angle_sample[0]:%lf\n", angle_sample[0]);
  printf("threshold:%lf\n", azimuth_threshold);
  printf("\n");
  //確認用の表示
  //for(cnt_angle_sample=0; cnt_angle_sample<number_of_cnt_ext_max_time; cnt_angle_sample++){
  //  printf("angle_sample:%lf\n", angle_sample[cnt_angle_sample]);
  //}

  for(cnt_angle_sample=0; cnt_angle_sample<=number_of_cnt_ext_max_time; cnt_angle_sample++){
    //printf("%lf\n", angle_sample[m]);
    if( cnt_angle_sample != 0 && angle_sample[cnt_angle_sample] != angle_sample[cnt_angle_sample-1] ){
      if( (azimuth_threshold<=angle_sample[cnt_angle_sample] && angle_sample[cnt_angle_sample]<azimuth_threshold+180) && cnt_azimuth_1_sample<AZIMUTH_SAMPLE_NUMBER ){
        azimuth_1_sample[cnt_azimuth_1_sample] = angle_sample[cnt_angle_sample];
        cnt_azimuth_1_sample ++;
        azimuth_1_norm = azimuth_1_norm + mag_sample[cnt_angle_sample];
      }

      if( (angle_sample[cnt_angle_sample]<azimuth_threshold || azimuth_threshold+180<=angle_sample[cnt_angle_sample]) && cnt_azimuth_2_sample<AZIMUTH_SAMPLE_NUMBER ){
        if(azimuth_threshold+180<=angle_sample[cnt_angle_sample]){
          azimuth_2_sample[cnt_azimuth_2_sample] = angle_sample[cnt_angle_sample] - 360;
        }else{
          azimuth_2_sample[cnt_azimuth_2_sample] = angle_sample[cnt_angle_sample];
        }
        cnt_azimuth_2_sample ++;
        azimuth_2_norm = azimuth_2_norm + mag_sample[cnt_angle_sample];
      }

    }
  }

  printf("average norm of azimuth_1:%lf azimuth_2:%lf\n", azimuth_1_norm/cnt_azimuth_1_sample, azimuth_2_norm/cnt_azimuth_2_sample);
  printf("\n");
  //それぞれの方位角を四捨五入して、表示する
  for(cnt_angle_sample=0; cnt_angle_sample<cnt_azimuth_1_sample;  cnt_angle_sample++){
    //azimuth_1_total = azimuth_1_total + (int)round(azimuth_1_sample[cnt_angle_sample]);
    azimuth_1_sample[cnt_angle_sample] = (int)round(azimuth_1_sample[cnt_angle_sample]);
    printf("%d ", (int)round(azimuth_1_sample[cnt_angle_sample]));
  }
  printf("\n");
  for(cnt_angle_sample=0; cnt_angle_sample<cnt_azimuth_2_sample; cnt_angle_sample++){
    //azimuth_2_total = azimuth_2_total + (int)round(azimuth_2_sample[cnt_angle_sample]);
    azimuth_2_sample[cnt_angle_sample] = (int)round(azimuth_2_sample[cnt_angle_sample]);
    printf("%d ", (int)round(azimuth_2_sample[cnt_angle_sample]));
  }
  printf("\n");

  //それぞれの方位角の平均を求める
  azimuth_1_average = get_average(azimuth_1_sample, cnt_azimuth_1_sample);
  azimuth_2_average = get_average(azimuth_2_sample, cnt_azimuth_2_sample);

  printf( "sample number: %d and %d\n", cnt_azimuth_1_sample, cnt_azimuth_2_sample );
  printf( "average: %d and %d\n", (int)round(azimuth_1_average), (int)round(azimuth_2_average) );

  //求めた方位角の分散を求める処理
  azimuth_1_deviation = get_deviation(azimuth_1_sample, cnt_azimuth_1_sample);
  azimuth_2_deviation = get_deviation(azimuth_2_sample, cnt_azimuth_2_sample);

  //printf( "deviation: %.2lf and %.2lf\n", azimuth_1_deviation, azimuth_2_deviation );
  printf( "deviation: %d and %d\n", (int)round(azimuth_1_deviation), (int)round(azimuth_2_deviation) );
  printf("\n");

  //fclose(fp3);
  fclose(fp_motor);

  if(azimuth_1_norm>azimuth_2_norm){
    printf("azimuth:%d\n", (int)round(azimuth_1_average));
  }else{
    printf("azimuth:%d\n", (int)round(azimuth_2_average));
  }
  printf("\n");

  return;
}

void elevation(char *sen_mag_file, char *mag_file, char *norm_file, char *motor_file){
  int cnt_mag_norm_data = 0;
  int l3 = 0;

  double psi_1[MAX_SAMPLE_DATA_NUMBER];
  double psi_2[MAX_SAMPLE_DATA_NUMBER];
  double psi_1_average = 0;
  double psi_2_average = 0;
  double psi_1_deviation = 0;
  double psi_2_deviation = 0;
  int cnt_psi_1 = 0;
  int cnt_psi_2 = 0;

  double h_time = 0;
  double h_x = 0;
  double h_y = 0;
  double h_z = 0;

  double h_x_max[MAX_SAMPLE_DATA_NUMBER];
  double h_z_max[MAX_SAMPLE_DATA_NUMBER];
  double h_y_max[MAX_SAMPLE_DATA_NUMBER];

  int cnt_h_max = 0;
  int cnt_h_min = 0;

  double cos_psi[MAX_SAMPLE_DATA_NUMBER];
  double psi[MAX_SAMPLE_DATA_NUMBER];

  file_read(&fp_mag, mag_file);//センサ値のx,y,z成分

  while(fscanf(fp_mag,"%lf,%lf,%lf,%lf\n", &h_time, &h_x, &h_y, &h_z)!=EOF) {
    //printf("time:%lf x:%lf y:%lf z:%lf\n", h_time, h_x, h_y, h_z);
    //printf("%d %d\n", cnt_h_max, cnt_h_min);
    for(l3=0;l3<=ext_max_time_number;l3++){

      if(ext_max_time[l3] - TIME_DELTA <= h_time && h_time <= ext_max_time[l3] + TIME_DELTA){
        //この辺はセンサー値が振動を始める前の揺らぎを極値として認識しないための処理
        //h_maxなどの必要性は？
        if( cnt_h_max == 0 && abs(h_x - x_min ) > abs(x_min) ){
          h_x_max[0] = h_x;
          h_z_max[0] = h_z;
          cnt_h_max++;
        }

        if( cnt_h_max != 0 && abs(h_x - h_x_max[cnt_h_max-1]) > abs(x_max) ){
          h_x_max[cnt_h_max] = h_x;
          h_z_max[cnt_h_max] = h_z;
          cnt_h_max ++;
        }
      }
    }

    for(l3=0;l3<=ext_min_time_number;l3++){
      if(ext_min_time[l3] - TIME_DELTA <= h_time && h_time <= ext_min_time[l3] + TIME_DELTA){

        if( cnt_h_min == 0 ){
          h_y_max[0] = h_y;
          cnt_h_min++;
        }

        if( cnt_h_min != 0 && abs(h_y - h_y_max[cnt_h_min-1]) > abs(y_max) ){
          h_y_max[cnt_h_min] = h_y;
          cnt_h_min ++;
        }

      }
    }
  }

  fclose(fp_mag);

  for(l3=0;l3<=cnt_h_max-1;l3++){
    printf("h_x:%lf h_z:%lf\n", h_x_max[l3], h_z_max[l3]);
  }
  printf("sample number:%d\n", cnt_h_max);
  printf("\n");

  for(l3=0;l3<=cnt_h_min-1;l3++){
    printf("h_y:%lf\n", h_y_max[l3]);
  }
  printf("sample number:%d\n", cnt_h_min);
  printf("\n");

  for(l3=0;l3<=cnt_h_max-1;l3++){
    cos_psi[l3] = (h_x_max[l3] + h_y_max[l3])/sqrt(h_z_max[l3]*h_z_max[l3]+(h_x_max[l3]+h_y_max[l3])*(h_x_max[l3]+h_y_max[l3]));
    psi[l3] = (180/PI)*acos( cos_psi[l3] );
    psi[l3] = (int)round(psi[l3]);
    //仰角のプラスマイナスを特定する処理、非対称磁石が使えるようになったら解禁
    /*if(h_z_max[l3]>0){
      psi[l3] = 1 * psi[l3];
    }else if(h_z_max[l3]<0){
      psi[l3] = (-1) * psi[l3];
    }*/
    //磁石のN極側が向いている時の仰角は９０°より小さくなり、S極側が向いている時は９０°より大きくなる（この辺り改良が必要）
    if( cos_psi[l3]>=0 ){
      if(cnt_psi_1 < ELEVATION_SAMPLE_NUMBER){
        if(h_z_max[l3]>0){
          psi[l3] = 1 * psi[l3];
        }else if(h_z_max[l3]<0){
          psi[l3] = (-1) * psi[l3];
        }
        psi_1[cnt_psi_1] = psi[l3];
        cnt_psi_1 ++;
      }
    }
    if( cos_psi[l3]<0 ){
      if(cnt_psi_2 < ELEVATION_SAMPLE_NUMBER){
        if(h_z_max[l3]<0){
          psi[l3] = 1 * psi[l3];
        }else if(h_z_max[l3]>0){
          psi[l3] = (-1) * psi[l3];
        }
        psi_2[cnt_psi_2] = psi[l3];
        cnt_psi_2 ++;
      }
    }
    /*if( cos_psi>=0 && (0<=psi[l3] && psi[l3]<90) ){
    //if(psi[l3]<90){
      if(cnt_psi_1 < ELEVATION_SAMPLE_NUMBER){
        if(h_z_max[l3]>0){
          psi[l3] = 1 * psi[l3];
        }else if(h_z_max[l3]<0){
          psi[l3] = (-1) * psi[l3];
        }
        //psi_1_average = psi_1_average + psi[l3];
        psi_1[cnt_psi_1] = psi[l3];
        cnt_psi_1 ++;
      }
    //}else if(psi[l3]>90){
    }else if( cos_psi>=0 && (-90<=psi[l3] && psi[l3]<0) ){
      if(cnt_psi_2 < ELEVATION_SAMPLE_NUMBER){
        if(h_z_max[l3]<0){
          psi[l3] = 1 * psi[l3];
        }else if(h_z_max[l3]>0){
          psi[l3] = (-1) * psi[l3];
        }
        //psi_2_average = psi_2_average + psi[l3];
        psi_2[cnt_psi_2] = psi[l3];
        cnt_psi_2 ++;
      }
    }
    if( cos_psi<0 && (90<=psi[l3] && psi[l3]<270) ){
      if(cnt_psi_1 < ELEVATION_SAMPLE_NUMBER){
        if(h_z_max[l3]<0){
          psi[l3] = 1 * psi[l3];
        }else if(h_z_max[l3]>0){
          psi[l3] = (-1) * psi[l3];
        }
        //psi_1_average = psi_1_average + psi[l3];
        psi_1[cnt_psi_1] = psi[l3];
        cnt_psi_1 ++;
      }
    }else if( cos_psi<0 && (-180<=psi[l3] && psi[l3]<-90) ){
      if(cnt_psi_2 < ELEVATION_SAMPLE_NUMBER){
        if(h_z_max[l3]>0){
          psi[l3] = 1 * psi[l3];
        }else if(h_z_max[l3]<0){
          psi[l3] = (-1) * psi[l3];
        }
        //psi_2_average = psi_2_average + psi[l3];
        psi_2[cnt_psi_2] = psi[l3];
        cnt_psi_2 ++;
      }
    }*/
    //printf("cos_psi: %lf   elevation: %d\n", cos_psi[l3], (int)psi[l3]);
    //printf("elevation: %lf\n", psi);
  }
  for(l3=0;l3<2*ELEVATION_SAMPLE_NUMBER;l3++){
    printf("cos_psi: %lf   elevation: %d\n", cos_psi[l3], (int)psi[l3]);
  }
  printf("sample number: %d and %d\n", cnt_psi_1, cnt_psi_2);

  //求めた仰角の平均と分散を求める処理
  psi_1_average = get_average(psi_1, cnt_psi_1);
  psi_2_average = get_average(psi_2, cnt_psi_2);
  psi_1_deviation = get_deviation(psi_1, cnt_psi_1);
  psi_2_deviation = get_deviation(psi_2, cnt_psi_2);
  printf("average: %d and %d\n", (int)psi_1_average, (int)psi_2_average);
  printf("deviation: %d and %d\n", (int)round(psi_1_deviation), (int)round(psi_2_deviation));
  printf("\n");

  return;
}

void orientation(char *mag_file, char *sen_acc_file, char *mag_file_2){
  int cnt_acc = 0;

  double acc_x[MAX_SAMPLE_DATA_NUMBER];
  double acc_y[MAX_SAMPLE_DATA_NUMBER];
  double acc_z[MAX_SAMPLE_DATA_NUMBER];
  double ave_acc_x = 0;
  double ave_acc_y = 0;
  double ave_acc_z = 0;
  double acc_norm = 0;

  double cos_alpha = 0;
  double cos_beta = 0;
  double cos_gamma = 0;
  double alpha = 0;
  double beta = 0;
  double gamma = 0;

  FILE *fp_mag;//磁気センサーのデータファイルのポインタ
  FILE *fp_acc;//加速度センサーのデータファイルのポインタ
  FILE *fp_mag_2;//加速度センサーから修正を施した後の磁気センサーのデータファイル

  file_read(&fp_acc, sen_acc_file);//加速度センサのデータファイルを読み込む
  while( ( ret = fscanf(fp_acc, "%lf, %lf, %lf, %lf, %lf\n", &f1, &f2, &f3, &f4, &f5) ) != EOF ){
    if(cnt_acc < MAX_SAMPLE_DATA_NUMBER){
      acc_x[cnt_acc] = f3;
      acc_y[cnt_acc] = f4;
      acc_z[cnt_acc] = f5;
      cnt_acc ++;
      //printf("cnt_acc:%d\n", cnt_acc);
    }
  }
  fclose(fp_acc);

  //加速度の各成分の平均を求める
  ave_acc_x = get_average(acc_x, cnt_acc);
  ave_acc_y = get_average(acc_y, cnt_acc);
  ave_acc_z = get_average(acc_z, cnt_acc);

  acc_norm = sqrt(pow(ave_acc_x,2)+pow(ave_acc_y,2)+pow(ave_acc_z,2));
  printf("\n");
  printf("acc_x:%lf acc_y:%lf acc_z:%lf\n", ave_acc_x, ave_acc_y, ave_acc_z);
  printf("acc_norm:%lf\n", acc_norm);
  printf("\n");

  //加速度ベクトルが
  cos_alpha = ( 1/sqrt(pow(ave_acc_y,2) + pow(ave_acc_z,2)) )*(-ave_acc_z);
  alpha = acos(cos_alpha);
  printf("cos_alpha:%lf alpha:%lf[radian] alpha:%lf[degree]\n", cos_alpha, alpha, radian_to_degree(alpha));
  //alpha = radian_to_degree(alpha);
  //printf("alpha:%lf(degree)\n", alpha);
  //printf("alpha:%lf(degree)\n", radian_to_degree(alpha));

  //y軸と加速度ベクトルが成す角度
  cos_beta = ( 1/sqrt(pow(ave_acc_x,2) + pow(ave_acc_z,2)) )*(-ave_acc_z);
  beta = acos(cos_beta);
  printf("cos_beta:%lf beta:%lf[radian] beta:%lf[degree]\n", cos_beta, beta, radian_to_degree(beta));
  //beta = radian_to_degree(beta);
  //printf("beta:%lf(degree)\n", beta);
  //printf("beta:%lf(degree)\n", radian_to_degree(beta));

  cos_gamma = ( 1/sqrt(pow(ave_acc_x,2) + pow(ave_acc_y,2)) )*ave_acc_x;
  gamma = acos(cos_gamma);
  printf("cos_gamma:%lf gamma:%lf[radian] gamma:%lf[degree]\n", cos_gamma, gamma, radian_to_degree(gamma));

  //rotate_x(mag_x, mag_y, mag_z, mag_data_number, gamma);//z軸との角度でx軸中心に回転させる
  //rotate_y(mag_x, mag_y, mag_z, mag_data_number, beta);//z軸との角度でy軸中心に回転させる
  rotate_z(mag_x, mag_y, mag_z, mag_data_number, alpha);//x軸との角度でz軸中心に回転させる

  file_write(&fp_mag_2, mag_file_2);
  int cnt_mag_data = 0;
  for(cnt_mag_data=0; cnt_mag_data<mag_data_number; cnt_mag_data++){
    fprintf(fp_mag_2, "%lf,%lf,%lf\n", mag_x[cnt_mag_data], mag_y[cnt_mag_data], mag_z[cnt_mag_data]);
  }
  fclose(fp_mag_2);

  return;
}



void file_read(FILE **fp, char *argv){
  *fp = fopen( argv, "r");
  if( *fp == NULL ){
    printf( "%sファイルが開けません¥n", argv );
  }
}

void file_write(FILE **fp, char *argv){
  *fp = fopen( argv, "w");
  if( *fp == NULL ){
    printf( "%sファイルが開けません¥n", argv );
  }
}

void smoothing(FILE *file , char *argv){
  int n = 0;
  int k = 0;
  int l = 0;
  int i = 0;

  double dx;
  double dy;
  double norm_smoothing[MAX_SMOOTHING_DATA_NUMBER];
  double unixtime[MAX_SAMPLE_DATA_NUMBER];
  double norm[MAX_SAMPLE_DATA_NUMBER];

  for(i=0; i<SMOOTHING_TIMES; i++){
    n = 0;

    file_read( &file, argv );

    while(fscanf(file,"%lf,%lf\n",&dx,&dy)!=EOF) {
      if(n < MAX_SMOOTHING_DATA_NUMBER){
        unixtime[n] = dx;
        norm[n] = dy;
        n++;
      }
    }

    fclose(file);

    file_write( &file, argv );

    n = MAX_SMOOTHING_DATA_NUMBER-1;

    norm_smoothing[0] = norm[0];
    norm_smoothing[n] = norm[n];

    for(k=1;k<n;k++) {
      norm_smoothing[k] = (norm[k-1]+norm[k]+norm[k+1])/3;
    }

    for(l=0;l<=n;l++){
      fprintf(file, "%lf,%lf\n", unixtime[l], norm_smoothing[l]);
    }

    fclose(file);

  }
}

void smoothing_xyz(FILE *file , char *argv){
  int n = 0;
  int k = 0;
  int l = 0;
  int i = 0;
  int cnt_sm_val_number = 0;
  int number_val = 0;

  double mag[5][MAX_SMOOTHING_DATA_NUMBER];
  double dmag[5];
  double intensity[5][MAX_SMOOTHING_DATA_NUMBER];

  double unixtime[MAX_SMOOTHING_DATA_NUMBER];
  double norm[MAX_SMOOTHING_DATA_NUMBER];
  double dt = 0;

  for(i=0; i<SMOOTHING_TIMES; i++){
    n = 0;
    cnt_sm_val_number = 0;

    file_read( &file, argv );

    while(fscanf(file,"%lf,%lf,%lf,%lf\n",&dt,&dmag[1],&dmag[2],&dmag[3])!=EOF) {
      if(n<MAX_SMOOTHING_DATA_NUMBER){
        //if(dt != 0){
          unixtime[n] = dt;
          mag[1][n] = dmag[1];
          mag[2][n] = dmag[2];
          mag[3][n] = dmag[3];
          n++;
          cnt_sm_val_number ++;
          //printf("%d %d %d\n", cnt_sm_val_number, n, i);
        //printf("%d\n",n);
        //}
      }
    }

    fclose(file);

    for( number_val=1;number_val<=3;number_val++){
      //n = MAX_SMOOTHING_DATA_NUMBER-1;
      n = cnt_sm_val_number - 1;
      //printf("%d   %d\n", n, number_val);

      intensity[number_val][0] = mag[number_val][0];
      intensity[number_val][n] = mag[number_val][n];

      for(k=1;k<n;k++) {
        intensity[number_val][k] = (mag[number_val][k-1]+mag[number_val][k]+mag[number_val][k+1])/3;
        //printf("%lf\n", intensity[number_val][k]-mag[number_val][k]);
      }
    }

    file_write( &file, argv );

    for(l=0;l<=n;l++){
      if(unixtime[l] != 0){
        fprintf(file, "%lf,%lf,%lf,%lf\n", unixtime[l], intensity[1][l], intensity[2][l], intensity[3][l]);
      }
    }

    fclose(file);
  }
}



double radian_to_degree(double angle){
  return (180/PI)*angle;
}

double get_average(double *data, int data_number){
  int cnt_data = 0;
  double average = 0;
  double total = 0;

  for(cnt_data=0; cnt_data<data_number; cnt_data++){
    total = total + data[cnt_data];
  }
  average = total/data_number;

  return average;
}

double get_max(double *data, int data_number){
  int cnt_data = 0;
  double average = 0;
  double data_max = 0;

  average = get_average(data, data_number);
  data_max = average;

  for(cnt_data=0; cnt_data<data_number; cnt_data++){
    if(data_max<data[cnt_data]){
      data_max = data[cnt_data];
    }
  }

  return data_max;
}

double get_min(double *data, int data_number){
  int cnt_data = 0;
  double average = 0;
  double data_min = 0;

  average = get_average(data, data_number);
  data_min = average;

  for(cnt_data=0; cnt_data<data_number; cnt_data++){
    if(data_min>data[cnt_data]){
      data_min = data[cnt_data];
    }
  }

  return data_min;
}

double get_deviation(double *data, int data_number){
  int cnt_data = 0;
  int cnt_deviation = 0;
  double data_average = 0;
  double data_deviation = 0;

  data_average = get_average(data, data_number);
  //printf("%lf\n", data_average);

  for(cnt_data=0; cnt_data<data_number; cnt_data++){
    data_deviation = data_deviation + pow( data[cnt_data] - data_average, 2 );
    cnt_deviation ++;
    //printf("%lf\n", data[cnt_data]);
  }
  data_deviation = sqrt( data_deviation/cnt_deviation );
  //printf("data_deviation:%lf\n", data_deviation);
  return data_deviation;
}



double get_median(double *data, int data_number){
  int cnt_data = 0;

  double ext_max_data[MAX_SAMPLE_DATA_NUMBER];//極大値のデータ
  double ext_min_data[MAX_SAMPLE_DATA_NUMBER];//極小値のデータ
  int ext_max_data_number = 0;//極大値の数
  int ext_min_data_number = 0;//極小値の数

  double ext_max_average = 0;//極大値の平均
  double ext_min_average = 0;//極小値の平均
  double median = 0;//極大値の平均と極小値の平均の中央値

  get_extreme_value(data, data_number, ext_max_data, &ext_max_data_number, ext_min_data, &ext_min_data_number);
  //printf("%d %d\n", ext_max_data_number, ext_min_data_number);
  //極大値の平均と極小値の平均を算出し、さらにそれらの中央値を求める
  //もし、明確な極大値、極小値が存在していなければ中央値には平均値を代入する
  if(ext_max_data_number>=AZIMUTH_SAMPLE_NUMBER || ext_min_data_number>=AZIMUTH_SAMPLE_NUMBER){
    for(cnt_data=0; cnt_data<ext_max_data_number; cnt_data++){
      ext_max_average = ext_max_average + ext_max_data[cnt_data];
    }
    ext_max_average = ext_max_average/ext_max_data_number;
    for(cnt_data=0; cnt_data<ext_min_data_number; cnt_data++){
      ext_min_average = ext_min_average + ext_min_data[cnt_data];
    }
    ext_min_average = ext_min_average/ext_min_data_number;
    median = (ext_max_average + ext_min_average)/2;
  }else{
    median = get_average(data, data_number);
    //for(cnt_data=0; cnt_data<data_number; cnt_data++){
    //  data[cnt_data] = 0;
    //  median = 0;
    //  printf("data:%lf\n", data[cnt_data]);
    //}
  }

  return median;
}

void get_extreme_value(double *data, int data_number, double *ext_max_data, int *ext_max_data_number, double *ext_min_data, int *ext_min_data_number){
  double d0 = 0;
  double d1 = 0;
  double d2 = 0;
  double d3 = 0;

  int cnt_data = 0;
  int cnt_ext_max_data = 0;
  int cnt_ext_min_data = 0;

  double data_average = 0;
  double data_max = 0;
  double data_min = 0;

  int flag_ext_max_data = 0;
  int flag_ext_min_data = 0;

  data_average = get_average(data, data_number);
  data_max = get_max(data, data_number);
  data_min = get_min(data, data_number);
  //printf("average:%lf max:%lf min:%lf\n", data_average, data_max, data_min);

  for(cnt_data=2; cnt_data<data_number-2; cnt_data++) {
    //注目している点から二つとなりまでのデータの値を比較
    d0 = data[cnt_data-1] - data[cnt_data-2];
    d1 = data[cnt_data] - data[cnt_data-1];
    d2 = data[cnt_data+1] - data[cnt_data];
    d3 = data[cnt_data+2] - data[cnt_data + 1];

    //極大値の候補を検出
    if( cnt_ext_max_data == 0 && fabs(data[cnt_data] - data_min ) > fabs(data_min - data_average) && d0>0 && d1>0 && d2<0 && d3<0 ){
      ext_max_data[0] = data[cnt_data];
      cnt_ext_max_data ++;
      flag_ext_max_data = 1;
      flag_ext_min_data = 0;
      //printf("max! %lf %d %d\n", ext_max_data[0], flag_ext_max_data, flag_ext_min_data);
    }
    if( flag_ext_max_data != 1 && flag_ext_min_data == 1 && cnt_ext_max_data != 0 && fabs(data[cnt_data] - ext_min_data[cnt_ext_min_data - 1] ) > fabs(data_min - data_average) && d0>0 && d1>0 && d2<0 && d3<0 ){
      ext_max_data[cnt_ext_max_data] = data[cnt_data];
      cnt_ext_max_data ++;
      flag_ext_max_data = 1;
      flag_ext_min_data = 0;
      //printf("max! %lf %d %d\n", ext_max_data[cnt_ext_max_data-1], flag_ext_max_data, flag_ext_min_data);
    }

    //極小値の候補を検出
    if( cnt_ext_min_data == 0 && fabs(data[cnt_data] - data_max ) > fabs(data_max - data_average) && d0<0 && d1<0 && d2>0 && d3>0 ){
      ext_min_data[0] = data[cnt_data];
      cnt_ext_min_data ++;
      flag_ext_min_data = 1;
      flag_ext_max_data = 0;
      //printf("min! %lf %d %d\n", ext_min_data[0], flag_ext_max_data, flag_ext_min_data);
    }
    if( flag_ext_min_data != 1 && flag_ext_max_data == 1 && cnt_ext_min_data != 0 && fabs(data[cnt_data] - ext_max_data[cnt_ext_max_data - 1]) > fabs(data_max - data_average) && d0<0 && d1<0 && d2>0 && d3>0 ){
      ext_min_data[cnt_ext_min_data] = data[cnt_data];
      cnt_ext_min_data ++;
      flag_ext_min_data = 1;
      flag_ext_max_data = 0;
      //printf("min! %lf %d %d\n", ext_min_data[cnt_ext_min_data-1], flag_ext_max_data, flag_ext_min_data);
    }
  }
  //最初と最後の極値のデータは誤検出の可能性があるため、最初と最後以外のデータを採用
  for(cnt_data=1; cnt_data<cnt_ext_max_data-1; cnt_data++){
    ext_max_data[cnt_data-1] = ext_max_data[cnt_data];
  }
  ext_max_data[cnt_data] = 0;
  for(cnt_data=1; cnt_data<cnt_ext_min_data-1; cnt_data++){
    ext_min_data[cnt_data-1] = ext_min_data[cnt_data];
  }
  ext_min_data[cnt_data] = 0;

  *ext_max_data_number = cnt_ext_max_data - 2;
  *ext_min_data_number = cnt_ext_min_data - 2;
  //printf("ext_max_number:%d ext_min_number:%d\n", cnt_ext_max_data, cnt_ext_min_data);
  printf("ext_max_number:%d ext_min_number:%d\n", *ext_max_data_number, *ext_min_data_number);

}

void get_extreme_norm_and_unixtime(double *data, int data_number, double *unixtime, double *ext_max_time, int *ext_max_time_number, double *ext_min_time, int *ext_min_time_number, double *ext_max_data, double *ext_min_data){
  double d0 = 0;
  double d1 = 0;
  double d2 = 0;
  double d3 = 0;

  int cnt_data = 0;
  int cnt_ext_max_data = 0;
  int cnt_ext_min_data = 0;

  double data_average = 0;
  double data_max = 0;
  double data_min = 0;

  int flag_ext_max_data = 0;
  int flag_ext_min_data = 0;

  data_average = get_average(data, data_number);
  data_max = get_max(data, data_number);
  data_min = get_min(data, data_number);
  printf("average:%lf max:%lf min:%lf\n", data_average, data_max, data_min);

  for(cnt_data=2; cnt_data<data_number-2; cnt_data++) {
    //注目している点から二つとなりまでのデータの値を比較
    d0 = data[cnt_data-1] - data[cnt_data-2];
    d1 = data[cnt_data] - data[cnt_data-1];
    d2 = data[cnt_data+1] - data[cnt_data];
    d3 = data[cnt_data+2] - data[cnt_data + 1];

    //極大値の候補を検出
    if( cnt_ext_max_data == 0 && (fabs(data[cnt_data] - data_min) > fabs(data_max - data_average)/FACTOR || fabs(data[cnt_data] - data_min) > fabs(data_average - data_min)/FACTOR) && d0>0 && d1>0 && d2<0 && d3<0 ){
      ext_max_data[0] = data[cnt_data];
      ext_max_time[0] = unixtime[cnt_data];

      cnt_ext_max_data ++;
      flag_ext_max_data = 1;
      flag_ext_min_data = 0;
      //printf("max! %lf %d %d\n", ext_max_data[0], flag_ext_max_data, flag_ext_min_data);
    }
    if( flag_ext_max_data != 1 && flag_ext_min_data == 1 && cnt_ext_max_data != 0 && (fabs(data[cnt_data] - ext_min_data[cnt_ext_min_data - 1] ) > fabs(data_max - data_average)/FACTOR || fabs(data[cnt_data] - ext_min_data[cnt_ext_min_data - 1] ) > fabs(data_average - data_min)/FACTOR)&& d0>0 && d1>0 && d2<0 && d3<0 ){
      ext_max_data[cnt_ext_max_data] = data[cnt_data];
      ext_max_time[cnt_ext_max_data] = unixtime[cnt_data];
      //printf("ext_max_time:%lf\n", ext_max_time[cnt_ext_max_data]);

      cnt_ext_max_data ++;
      flag_ext_max_data = 1;
      flag_ext_min_data = 0;
      //printf("max! %lf %d %d\n", ext_max_data[cnt_ext_max_data-1], flag_ext_max_data, flag_ext_min_data);
    }

    //極小値の候補を検出
    if( cnt_ext_min_data == 0 && ( fabs(data_max - data[cnt_data]) > fabs(data_average - data_min )/FACTOR || fabs(data_max - data[cnt_data]) > fabs(data_average - data_min)/FACTOR ) && d0<0 && d1<0 && d2>0 && d3>0 ){
      ext_min_data[0] = data[cnt_data];
      ext_min_time[0] = unixtime[cnt_data];
      //printf("ext_min_time:%lf\n", ext_min_time[cnt_ext_min_data]);

      cnt_ext_min_data ++;
      flag_ext_min_data = 1;
      flag_ext_max_data = 0;
      //printf("min! %lf %d %d\n", ext_min_data[0], flag_ext_max_data, flag_ext_min_data);
    }
    if( flag_ext_min_data != 1 && flag_ext_max_data == 1 && cnt_ext_min_data != 0 && ( fabs(ext_max_data[cnt_ext_max_data - 1] - data[cnt_data]) > fabs(data_max - data_average)/FACTOR || fabs(ext_max_data[cnt_ext_max_data - 1] - data[cnt_data]) > fabs(data_average - data_min)/FACTOR ) && d0<0 && d1<0 && d2>0 && d3>0 ){
      ext_min_data[cnt_ext_min_data] = data[cnt_data];
      ext_min_time[cnt_ext_min_data] = unixtime[cnt_data];
      //printf("ext_min_time:%lf\n", ext_min_time[cnt_ext_max_data]);

      cnt_ext_min_data ++;
      flag_ext_min_data = 1;
      flag_ext_max_data = 0;
      //printf("min! %lf %d %d\n", ext_min_data[cnt_ext_min_data-1], flag_ext_max_data, flag_ext_min_data);
    }
  }
  //最初と最後の極値のデータは誤検出の可能性があるため、最初と最後以外のデータを採用
  //確認用の表示
  //for(cnt_data=0; cnt_data<cnt_ext_max_data; cnt_data++){
  //  printf("ext_max_time:%lf\n", ext_max_time[cnt_data]);
  //}
  for(cnt_data=1; cnt_data<cnt_ext_max_data; cnt_data++){
    ext_max_time[cnt_data-1] = ext_max_time[cnt_data];
    //printf("ext_max_time:%lf\n", ext_max_time[cnt_data-1]);
  }
  for(cnt_data=1; cnt_data<cnt_ext_min_data; cnt_data++){
    ext_min_time[cnt_data-1] = ext_min_time[cnt_data];
  }

  for(cnt_data=1; cnt_data<cnt_ext_max_data; cnt_data++){
    ext_max_data[cnt_data-1] = ext_max_data[cnt_data];
  }
  for(cnt_data=1; cnt_data<cnt_ext_min_data; cnt_data++){
    ext_min_data[cnt_data-1] = ext_min_data[cnt_data];
  }

  ext_max_time[cnt_ext_max_data-1] = 0;
  ext_max_time[cnt_ext_max_data-2] = 0;
  ext_max_data[cnt_ext_max_data-1] = 0;
  ext_max_data[cnt_ext_max_data-2] = 0;

  ext_min_time[cnt_ext_max_data-1] = 0;
  ext_min_time[cnt_ext_max_data-2] = 0;
  ext_min_data[cnt_ext_min_data-1] = 0;
  ext_min_data[cnt_ext_min_data-2] = 0;
  //確認用の表示
  printf("\n");
  for(cnt_data=0; cnt_data<cnt_ext_max_data; cnt_data++){
    printf("ext_max_time:%lf\n", ext_max_time[cnt_data]);
  }
  printf("\n");
  for(cnt_data=0; cnt_data<cnt_ext_max_data; cnt_data++){
    printf("ext_min_time:%lf\n", ext_min_time[cnt_data]);
  }
  printf("\n");

  *ext_max_time_number = cnt_ext_max_data - 2;
  *ext_min_time_number = cnt_ext_min_data - 2;
  //printf("ext_max_number:%d ext_min_number:%d\n", cnt_ext_max_data, cnt_ext_min_data);
  //printf("ext_max_time_number:%d\n", *ext_max_time_number);
  printf("ext_max_time_number:%d ext_min_time_number:%d\n", *ext_max_time_number, *ext_min_time_number);
  //確認用の表示
  printf("\n");
  for(cnt_data=0; cnt_data<cnt_ext_max_data; cnt_data++){
    printf("ext_max_norm:%lf\n", ext_max_data[cnt_data]);
  }
  printf("\n");
  for(cnt_data=0; cnt_data<cnt_ext_min_data; cnt_data++){
    printf("ext_min_norm:%lf\n", ext_min_data[cnt_data]);
  }
  printf("\n");

}



void rotate_x(double *data_x, double* data_y, double* data_z, int data_number, double angle){
  int cnt_data = 0;
  for(cnt_data=0; cnt_data<data_number; cnt_data++){
    data_x[cnt_data] = data_x[cnt_data];
    data_y[cnt_data] = cos(angle)*data_y[cnt_data] - sin(angle)*data_z[cnt_data];
    data_z[cnt_data] = sin(angle)*data_y[cnt_data] + cos(angle)*data_z[cnt_data];
  }
}

void rotate_y(double *data_x, double* data_y, double* data_z, int data_number, double angle){
  int cnt_data = 0;
  for(cnt_data=0; cnt_data<data_number; cnt_data++){
    data_x[cnt_data] = cos(angle)*data_x[cnt_data] - sin(angle)*data_z[cnt_data];
    data_y[cnt_data] = data_y[cnt_data];
    data_z[cnt_data] = sin(angle)*data_x[cnt_data] + cos(angle)*data_z[cnt_data];
  }
}

void rotate_z(double *data_x, double* data_y, double* data_z, int data_number, double angle){
  int cnt_data = 0;
  for(cnt_data=0; cnt_data<data_number; cnt_data++){
    data_x[cnt_data] = cos(angle)*data_x[cnt_data] - sin(angle)*data_y[cnt_data];
    data_y[cnt_data] = sin(angle)*data_x[cnt_data] + cos(angle)*data_y[cnt_data];
    data_z[cnt_data] = data_z[cnt_data];
  }
}



void test(){
  int i = 0;
  double test[] = {1,2,3,4,5};
  double ave_test = get_average(test, 5);
  printf("%lf\n", ave_test);
  ave_test = 0;
  for(i=0; i<5; i++){
    ave_test = ave_test + test[i];
  }
  printf("%lf %d\n", ave_test/i, i );
}

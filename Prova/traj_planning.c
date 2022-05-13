#include <stdio.h>
#include <stdbool.h>
#include <time.h>
#include <stdlib.h>
#include <math.h>
#include <Polyhedron.h>
#define TXT_MAX_LEN 110
#define Tot_iterations 50000

typedef struct{
    double x;
    double y;
    double z;
    bool visited;
}Pose;

typedef struct {
    Pose pos[TXT_MAX_LEN];
    int len;
}Mappa;

typedef struct{
    int n_visitati;
    Mappa mappa;
    double distanza_percorsa;
    Pose actual_pose;
    int percorso[TXT_MAX_LEN];
} Zech;

Zech find_distance_to_next_point(Zech zech);

void salva_mappa(Mappa *map){
    FILE *io;
    
    
    if((io = fopen("my_cloud.xyz","r"))==NULL){
        printf("Errore nell’apertura del file!");
        return;
    }
    double x,y,z;
    int i=0;
    while (fscanf(io,"%lf %lf %lf",&x,&y,&z)==3)
    {

        map->pos[i].x=x;
        map->pos[i].y=y;
        map->pos[i].z=z;
        map->pos[i].visited=false;
        i++;
    }
    map->len=i;
    
    fclose(io);
}

void stampa_mappa(Mappa map){
    printf("Sto per stampare la mappa:\n");
    for(int i=0;i<map.len;i++){
        printf("%lf %lf %lf %d\n",map.pos[i].x,map.pos[i].y,map.pos[i].z,map.pos[i].visited);
    }
}

void Stampa_percorso(Zech zech){
    int target_index=0;
    Pose pos;
    for(int i=0;i<zech.mappa.len;i++){
        target_index=zech.percorso[i];
        pos=zech.mappa.pos[target_index];
        printf("%f %f %f\n",pos.x,pos.y,pos.z);
    }
}

void Salva_percorso_su_file(Zech zech){
    FILE *io;
    io=fopen("percorso.xyz","w");
    int target_index=0;
    Pose pos;
    for(int i=0;i<zech.mappa.len;i++){
        target_index=zech.percorso[i];
        pos=zech.mappa.pos[target_index];
        fprintf(io,"%f %f %f\n",pos.x,pos.y,pos.z);
    }
    fclose(io);
}
double compute_euclidean_distance(Pose p1,Pose p2){
    double dist_x=(p1.x-p2.x)*(p1.x-p2.x);
    double dist_y=(p1.y-p2.y)*(p1.y-p2.y);
    double dist_z=(p1.z-p2.z)*(p1.z-p2.z);
    return sqrt((double)(dist_x+dist_y+dist_z));

}

Zech find_distance_to_next_point(Zech zech){
    
    if(zech.n_visitati>=zech.mappa.len){
        //if(zech.n_visitati==5 && zech.distanza_percorsa>=0.39 && zech.distanza_percorsa<=0.41){
        //printf("n_visitati:%d dist=%f tot_dist=%f actual=%f next=%f\n",zech.n_visitati,distance,zech.distanza_percorsa,zech.actual_pose.x,new_point.x);
	//printf("%d %d %d %d %d\n",zech.percorso[0],zech.percorso[1],zech.percorso[2],zech.percorso[3],zech.percorso[4]);
	//}
        return zech;
    }

    int index;
    index=rand()%(zech.mappa.len);
    //printf("Index:%d  visitati:%d len:%d iterations:%d\n",index,zech.n_visitati,zech.mappa.len,zech.max_iterations);
    if(zech.mappa.pos[index].visited==false){
      zech.percorso[zech.n_visitati]=index;
      zech.n_visitati++;
      zech.mappa.pos[index].visited=true;
      Pose new_point=zech.mappa.pos[index];
      double distance=compute_euclidean_distance(new_point,zech.actual_pose);
    
      zech.distanza_percorsa+=distance;
      
      zech.actual_pose=new_point;
    }


    return find_distance_to_next_point(zech);
}

int main()
{


    return 0;


    srand(time(NULL));

    Mappa map;
    salva_mappa(&map);
	

    //stampa_mappa(map);

    Zech zech;
    zech.mappa=map;
    zech.actual_pose=map.pos[0];
    zech.mappa.pos[0].visited=true;
    zech.distanza_percorsa=0;
    zech.n_visitati=1;

    for(int i=0;i<map.len;i++){
      zech.percorso[i]=0;
    }

    printf("Sto per calcolare dist\n");
    double total_d=0;
    double min=100000;

    Zech best_zech,zech_trovato;

    for(int i=0;i<Tot_iterations;i++){
        zech_trovato=find_distance_to_next_point(zech);
        total_d=zech_trovato.distanza_percorsa;
        if(total_d<min){
            min=total_d;
            best_zech=zech_trovato;
}
        if(i%5000==0)
            printf("Iteration number:%d\n",i);
    }

    printf("Distanza totale: %lf\n",min);
    Stampa_percorso(best_zech);
    Salva_percorso_su_file(best_zech);
    return true;
}

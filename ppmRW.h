#ifndef PPMRW_H
#define PPMRW_H

#include <stdio.h>
#include <stdlib.h>
#include <time.h> // On include time pour pouvoir nommer les fichiers en fonction de l'heure à laquelle on les crée.
#include "generationParameters.h"

// cette fonction lis un fichier ppm
void read_ppm_sol(double matrix[heightTextureSol][widthTextureSol][3]) {
    char filename[] = "sol.ppm";  // Remplace par le chemin de ton fichier PPM

    FILE *f = fopen(filename, "r");

    char magic[3];
    int width, height, max_val;
    fscanf(f, "%2s %d %d %d", magic, &width, &height, &max_val);

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int r, g, b;
            fscanf(f, "%d %d %d", &r, &g, &b);

            matrix[y][x][0] = (double)r;
            matrix[y][x][1] = (double)g;
            matrix[y][x][2] = (double)b;
        }
    }

    fclose(f);
    printf("Sol créé\n");
}

// cette fonction lis un fichier ppm pour la planète
void read_ppm_planete(double matrix[heightTexturePlanete][widthTexturePlanete][3]) {
    char filename[] = "terre.ppm"; 

    FILE *f = fopen(filename, "r");

    char magic[3];
    int width, height, max_val;
    fscanf(f, "%2s %d %d %d", magic, &width, &height, &max_val);

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int r, g, b;
            fscanf(f, "%d %d %d", &r, &g, &b);

            matrix[y][x][0] = (double)r;
            matrix[y][x][1] = (double)g;
            matrix[y][x][2] = (double)b;
        }
    }

    fclose(f);

    printf("Terre créée\n");
}

// cette fonction écrit l'image de sortie dans un fichier ppm
void write_ppm(double matrix[Vh][Vw][3]) {

    time_t rawtime;
    struct tm *timeinfo;
    char buffer[80];

    time(&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer, sizeof(buffer), "%Y-%m-%d_%H-%M-%S", timeinfo);
    char filename[80];
    sprintf(filename,"C:/Users/paula/Desktop/nvray/rayTracing/rendus/rendu%s.ppm", buffer);

    printf("%s\n",filename);

    FILE *f = fopen(filename, "w");

    int height = Vh;
    int width = Vw;

    fprintf(f, "P3\n");
    fprintf(f, "%d %d\n", width, height);
    fprintf(f, "255\n");

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int r = (int)(matrix[y][x][0]);
            int g = (int)(matrix[y][x][1]);
            int b = (int)(matrix[y][x][2]);

            fprintf(f,"%d ",r);
            fprintf(f,"%d ",g);
            fprintf(f,"%d\n",b);
        }
    }

    fclose(f);
}

void write_ppm_idx(double matrix[Vh][Vw][3], int temps) {

    char filename[80];
    sprintf(filename,"C:/Users/paula/Desktop/nvray/rayTracing/rendus/rendu_%d.ppm", temps);

    printf("%s\n",filename);

    FILE *f = fopen(filename, "w");

    int height = Vh;
    int width = Vw;

    fprintf(f, "P3\n");
    fprintf(f, "%d %d\n", width, height);
    fprintf(f, "255\n");

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int r = (int)(matrix[y][x][0]);
            int g = (int)(matrix[y][x][1]);
            int b = (int)(matrix[y][x][2]);

            fprintf(f,"%d ",r);
            fprintf(f,"%d ",g);
            fprintf(f,"%d\n",b);
        }
    }

    fclose(f);
}

void clearImage(double matrix[Vh][Vw][3]){
    for (int y = 0; y < Vh; y++) {
        for (int x = 0; x < Vw; x++) {
            for (int color = 0; color < 3; color++){
                matrix[y][x][color]=0;
            }
        }
    }
}

#endif
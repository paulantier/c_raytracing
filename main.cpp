
// On inclut nos header ainsi que pthread pour le multithreading

#include "rayGeometry.h" // Contient les fonctions de calculs géométrique pour le lancer de rayon
#include "ppmRW.h" // Contient les fonctions de lecture écriture de fichiers ppm
#include "createEnv.h" // Contient les fonctions de créations des objets de notre espace (sphères, sol etc)
#include "colorGeneration.h" // Contient les algorithmes de génération d'absorption pour différents cas
#include "generationParameters.h" // Contient toutes les constantes modulables pour générer notre image.

#include "pthread.h" // Multithreading en C


// On définit deltaU et deltaV qui sont les écarts horizontaux et verticaux entre deux pixels du viewport dans le repère monde.
double deltaU=1.0 / Vw;
double deltaV=1.0 / Vh;

// On définit l'image résultat
double I[Vh][Vw][3];


double pixelValue[3]; //tableau qui stocke la sortie RGB pour chaque lancer de rayon
double R0[3]={0,0,0}; // Position de la caméra

int main()
{
    read_ppm_sol(imgAbsorptionTextureSol); // On génère un tableau correspondant à l'image de notre texture de sol
    read_ppm_planete(imgAbsorptionTexturePlanete);// On génère un tableau correspond à une image de planisphère de la terre
    creerSol(); // On créer un vecteur qui contient l'équation de notre plan
    creerPlanete(); // On crée les propriétés de notre Planete
    clearImage(I); // On crée la matrice I qui contiendra notre image de sortie
    creerSpheres(70); // On crée les sphères de notre image, le paramètre de la fonction est le temps car certaines sphères peuvent
    // se déplace en fonction de ce paramètre pour en faire une vidéo par exemple

    // On peut choisir pour chaque pixel de lancer plusieurs rayons pour moyenner les effets d'aléatoire et avoir une image plus lisse
    for (int noLancer = 0; noLancer < nbLancers; ++noLancer) {  
        // Pour chaque pixel de notre viewport, c'est à dire pour chaque pixel de notre image de sortie on fait :       
        for (int iPixel = 0; iPixel < Vh; iPixel++) {
            for (int jPixel = 0; jPixel < Vw; jPixel++) {

                // On calcule un point du viewport 
                double pij[3] = {-1.0f / 2, 1.0f / 2, -focale};
                pij[0] += jPixel * deltaU;
                pij[1] -= iPixel * deltaV;
                double norme_Rd=sqrt( (pij[0]-R0[0])*(pij[0]-R0[0]) + (pij[1]-R0[1])*(pij[1]-R0[1]) + (pij[2]-R0[2])*(pij[2]-R0[2]) );
                double Rd[3];
                for (int i = 0; i < 3; ++i) {
                    Rd[i] = (pij[i] - R0[i]) / norme_Rd;
                }

                // A partir de ce point du viewport on créé un vecteur directeur normalisé correspond au rayon que l'on va envoyer pour ce pixel.
                // On lance le rayon, fonction récursive qui sort une valeur flottante entre 0 et 255.

                // Le principe est de déterminer d'où viendrait un rayon qui arrive jusqu'à ce pixel de notre caméra en simulant la trajectoire inverse d'un tel rayon
                double* pixelValue_ptr = lancerRayon(R0, Rd, nbRebondsMax, Nespace, Spheres, Planete, eqPlan);
                for (int i = 0; i < 3; i++) {
                    double pixelValTemp= *(pixelValue_ptr + i);
                    pixelValue[i]=pixelValTemp;
                }
                
                // On colore notre pixel de la couleur obtenue par le lancer de rayon
                // On divise par le nombre de lancers dans le cas où on lance plusieurs rayons par pixels pour avoir toujours des valeurs entre 0 et 255;
                // Cette valeur sera arrondie en entier par la suite pour la création d'image.
                I[iPixel][jPixel][0] += pixelValue[0] / nbLancers;
                I[iPixel][jPixel][1] += pixelValue[1] / nbLancers;
                I[iPixel][jPixel][2] += pixelValue[2] / nbLancers;
            }
        }
    }
    // On génère une image PPM (portable pixmap) à partir de notre matrice RGB
    write_ppm(I);
    return 0;
}



/*

Cette version du main est exactement la même la précédentes sauf qu'elle rajoute une boucle de temps pour pouvoir générer une vidéo avec des sphères qui bougent.

int main()
{
    read_ppm_sol(imgAbsorptionTextureSol);
    read_ppm_planete(imgAbsorptionTexturePlanete);
    creerSol();
    creerPlanete();
    for (int temps = 0; temps<100; ++temps){
        clearImage(I);
        creerSpheres((double)temps);
        for (int noLancer = 0; noLancer < nbLancers; ++noLancer) {                 
            for (int iPixel = 0; iPixel < Vh; iPixel++) {
                for (int jPixel = 0; jPixel < Vw; jPixel++) {
                    double pij[3] = {-1.0f / 2, 1.0f / 2, -focale};
                    pij[0] += jPixel * deltaV;
                    pij[1] -= iPixel * deltaU;
                    double norme_Rd=sqrt( (pij[0]-R0[0])*(pij[0]-R0[0]) + (pij[1]-R0[1])*(pij[1]-R0[1]) + (pij[2]-R0[2])*(pij[2]-R0[2]) );
                    double Rd[3];
                    for (int i = 0; i < 3; ++i) {
                        Rd[i] = (pij[i] - R0[i]) / norme_Rd;
                    }

                    double* pixelValue_ptr = lancerRayon(R0, Rd, nbRebondsMax, Nespace, Spheres, Planete, eqPlan);
                    for (int i = 0; i < 3; i++) {
                        double pixelValTemp= *(pixelValue_ptr + i);
                        pixelValue[i]=pixelValTemp;
                    }

                    I[iPixel][jPixel][0] += pixelValue[0] / nbLancers;
                    I[iPixel][jPixel][1] += pixelValue[1] / nbLancers;
                    I[iPixel][jPixel][2] += pixelValue[2] / nbLancers;
                }
            }
        }
        //write_ppm(I);
        write_ppm_idx(I,temps);
    }
    return 0;
}
*/


/*

Cette section correspond au main identique à celui du haut de ce fichier mais avec du multithreading.
Cependant cette section reste une ébauche car il y a un problème de réentrance que je n'ai pas eu le temps de corriger.
En effet les threads fonctionnent parfaitement séparément mais ils doivent écrire dans le même tableau dans certains header donc
il y a des problèmes de null dans des appels de tableau donc des segmentation fault.

J'ai décidé d'opérer le multithreading en découpant l'image résultat en sections égales (par exemple en quarts) et en générant les différentes sections
de l'image en parallèle.

// On définit les arguments qu'on donne à notre thread dans un struct car un thread ne prend en entrée qu'un seul paramètres (ici notre struct defCadre)
struct defCadre {
    int iPixelMin;
    int iPixelMax;
    int jPixelMin;
    int jPixelMax;
};


// morceaudImage est la fonction appelée par notre thread, elle contient la double boucle for du main mais cette fois ci entre quatres coordonnées :
iMin,iMax,jMin et jMax qui définissent la zone dimage générée (les coordonnées des pixels entre lesquels ont lance les rayons).


void* morceaudImage(void* borduresCadre) {

    struct defCadre *cpBorduresCadre = (struct defCadre *)borduresCadre;
    int iPixelMinT=cpBorduresCadre->iPixelMin;
    int iPixelMaxT=cpBorduresCadre->iPixelMax;
    int jPixelMinT=cpBorduresCadre->jPixelMin;
    int jPixelMaxT=cpBorduresCadre->jPixelMax;

    for (int noLancer = 0; noLancer < nbLancers; noLancer++){
        for (int iPixel = iPixelMinT; iPixel < iPixelMaxT; iPixel++) {
            for (int jPixel = jPixelMinT; jPixel < jPixelMaxT; jPixel++) {
    
                double pij[3] = {-1.0f / 2, 1.0f / 2, -focale};
                pij[0] += jPixel * deltaV;
                pij[1] -= iPixel * deltaU;
                double norme_Rd=sqrt( (pij[0]-R0[0])*(pij[0]-R0[0]) + (pij[1]-R0[1])*(pij[1]-R0[1]) + (pij[2]-R0[2])*(pij[2]-R0[2]) );
                double Rd[3];
                for (int i = 0; i < 3; ++i) {
                    Rd[i] = (pij[i] - R0[i]) / norme_Rd;
                }

            double* pixelValue_ptr = lancerRayon(R0, Rd, nbRebondsMax, Nespace, Spheres, Planete, eqPlan);
                for (int i = 0; i < 3; i++) {
                    double pixelValTemp= *(pixelValue_ptr + i);
                    pixelValue[i]=pixelValTemp;
                }

                I[iPixel][jPixel][0] += pixelValue[0] / nbLancers;
                I[iPixel][jPixel][1] += pixelValue[1] / nbLancers;
                I[iPixel][jPixel][2] += pixelValue[2] / nbLancers;
            }
        }
    }

    pthread_exit(NULL); // On sort du thread après un seul passage car la boucle for est dans le thread.
    return NULL;
}

// Le main est donc changé lui car il faut simplement appeler les différents threads (le code est adapté pour qu'on puisse utiliser n'importe quel nombre
de thread tant que c'est le carré d'un entier : 1,4,9,25 etc).
J'ai inclus une clock pour vérifier l'impact des threads sur le code mais malheureusement si on utilise les thread en parallèle on a des problèmes
(J'avais créé les threads l'un après l'autre pour vérifier et ça marche bien).

#include <time.h>
int main()
{
    // On crée notre environnement
    read_ppm_sol(imgAbsorptionTextureSol);
    read_ppm_planete(imgAbsorptionTexturePlanete);
    creerSol();
    creerPlanete();
    clearImage(I);
    creerSpheres(40);

    // On crée notre clock et on la démarre
    clock_t start, end;
    double cpu_time_used;
    start = clock();
    
    On crée une liste de threads de dimension voulue
    const int numThreads = gridSizeThreads*gridSizeThreads;
    pthread_t threads[numThreads];
    defCadre cadreCoordonnees[numThreads];
    // On trouve la taille de la section d'image donc s'occupe chaque thread
    int tailleSectionGrid = Vh/gridSizeThreads;
    // On détermine les pixels limites de la zone que devra générer chaque thread :
    for (int iThread = 0; iThread < gridSizeThreads; iThread++){
        for (int jThread = 0; jThread < gridSizeThreads; jThread++){

            int idxThread=iThread*gridSizeThreads+jThread;
            
            cadreCoordonnees[idxThread].iPixelMin = tailleSectionGrid * iThread;
            cadreCoordonnees[idxThread].iPixelMax = tailleSectionGrid*(iThread+1);
            cadreCoordonnees[idxThread].jPixelMin = tailleSectionGrid * jThread;
            cadreCoordonnees[idxThread].jPixelMax = tailleSectionGrid*(jThread+1);

            
        }
    }
    
    // On crée les thread
    for(int idxThread = 0; idxThread < numThreads; idxThread++){
        pthread_create(&threads[idxThread], NULL, morceaudImage, (void*)&cadreCoordonnees[idxThread]);
        
    }

    // On lance les threads
    for(int idxThread = 0; idxThread < numThreads; idxThread++){
        pthread_join(threads[idxThread], NULL);
    }

    end=clock();
    double time_used = ((double) (end - start)) / CLOCKS_PER_SEC; // Calcul du temps écoulé en secondes
    printf("Temps d'execution: %f secondes\n", time_used);
    // On affiche le temps de calcul

    write_ppm(I);
    return 0;
}

*/


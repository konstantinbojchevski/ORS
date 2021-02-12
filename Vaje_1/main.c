#include <stdint.h> // Knjižnica standardnih tipov
                    // glej sekcijo Podatkovni tipi

#include <math.h>  // -lm 
#include<stdio.h>


/*
Odgovor:
    1.  uint8_t se v HIPu prevaja kot podatek tipa .byte
         
            |   C jezik       |   HIP zbirni jezik  |
   --------------------------------------------------- 
    uint8_t |  r1 = r2 << 3;  |   srli r2, r1, #3   |  Logični pomik v desno
   --------------------------------------------------- 
    int8_t  |  r1 = r2 << 3;  |   srai r2, r1, #3   |  Aritmetični pomik v desno            

    2. 
       a)  7 & 3 -> Bitni IN      b)  7 && 3 -> Logični in   
        0111                            ker je 7 != 0 kompajler prevaja 7 kot True 
     |  0011                            ker je 3 != 0 kompajler prevaja 3 kot True 
     -------                        
        0011 = 3                        True && True = True = 1

    3. a) Koda je neveljavna ker hočete spremeniti naslov spremenljivke i.
       b) Na pomilniški lokaciji z naslovom 3 boste vpisali vsebino spremenljivke i


*/ 

// 4a 

void reset_bit(uint32_t* x, uint8_t p);

// 4b 
void reset_two_bits(uint32_t* x, uint8_t p);

// 4c 
void set_bit(uint32_t* x, uint8_t p);

// 4d 
void set_two_bits(uint32_t* x, uint8_t p,uint8_t n);

// 4e
struct vector {
    int32_t x,y;
    float length;
};

void vector_length(struct vector *a);

void main(){

    //4_a
    printf("Reset 2. bit \n");
    uint32_t a = 0x0000000F;
    printf("Before: %u (DEC) 0x%X (HEX) \n",a,a);
    reset_bit(&a,2); // Prvi argument je naslov spremenljivke !!!!!
    printf("After:  %u (DEC) 0x%X (HEX) \n \n",a,a);

    //4_b
    printf("Reset 2. and 3. bit \n");
    a = 0x000000FF;
    printf("Before: %u (DEC) 0x%X (HEX) \n",a,a);
    reset_two_bits(&a,2); // Prvi argument je naslov spremenljivke !!!!!
    printf("After:   %u (DEC) 0x%X (HEX) \n \n",a,a);

    //4_c
    printf("Set 2. bit to value 1 \n");
    a = 0x0000000E;
    printf("Before: %u (DEC) 0x%X (HEX) \n",a,a);
    set_bit(&a,2); // Prvi argument je naslov spremenljivke !!!!!t
    printf("After:  %u (DEC) 0x%X (HEX) \n \n",a,a);

    //4_d
    printf("Set 2. and 3. bit to value 3 (11 binary) \n");
    a = 0x000000EF;
    printf("Before: %u (DEC) 0x%X (HEX) \n",a,a);
    set_two_bits(&a,2,3); // Prvi argument je naslov spremenljivke !!!!!
    printf("After: %u (DEC) 0x%X (HEX) \n \n",a,a);

    //4_d
    printf("Calculate length of vector \n");
    struct vector b;
    b.x = 4;
    b.y = -2;
    vector_length(&b);
    printf("Vector coordinates: (%d,%d) -> Length: (%.2f) \n",b.x,b.y,b.length);

}

void reset_bit(uint32_t* x, uint8_t p){
/*  Primer: resetiraj  bit z indeksom 2  

    ~    0  0  0  0  0  1  0  0   (1 << 2)
-------------------------------
         1  1  1  1  1  0  1  1   ~(1 << 2)


        x7 x6 x5 x4 x3 x2 x1 x0   
    &    1  1  1  1  1  0  1  1   (~(1 << 2)) 
--------------------------------
        x7 x6 x5 x4 x3  0 x1 x0
*/
    
    *x = *x & ~(1 << p);

}

void reset_two_bits(uint32_t* x, uint8_t p){
/*  Primer: resetiraj  bita z indeksoma 2 in 3
    
    ~    0  0  0  0  1  1  0  0   (3 << 2)
-------------------------------
         1  1  1  1  0  0  1  1   ~(3 << 2)


        x7 x6 x5 x4 x3 x2 x1 x0   
    &    1  1  1  1  0  0  1  1   (~(3 << 2)) 
--------------------------------
        x7 x6 x5 x4  0  0 x1 x0

*/
    *x = *x & ~(3 << p);

}


void set_bit(uint32_t* x, uint8_t p){
/*  Primer: nastavi bit z indeksom 2 na vrednost 

        x7 x6 x5 x4 x3 x2 x1 x0   
    |    0  0  0  0  0  1  0  0   (1 << 2)
--------------------------------
        x7 x6 x5 x4 x3  1  x1 x0
*/
    *x = *x | (1 << p);

}

void set_two_bits(uint32_t* x, uint8_t p, uint8_t n){
/* nastavi  biti z indeksoma 3 in 2 na n
            n je 0,1,2 ali 3
            n je dvobitna vrednost -> n[1] višji bit
                                   -> n[0] nižji bit

1. korak: resetiraj biti z indeksoma 3 in 2

        x7 x6 x5 x4 x3 x2 x1 x0   
    &    1  1  1  1  0  0  1  1   (~(3 << 2)) 
--------------------------------
        x7 x6 x5 x4  0  0 x1 x0    (TMP)

2. korak: nastavi bit z indeksom 2

        x7 x6 x5 x4    0    0     x1 x0   (TMP)
    |    0  0  0  0  n[1] n[0]     0  0   (n << 2) 
--------------------------------
        x7 x6 x5 x4  n[1] n[0]  x1 x0
*/
    uint32_t tmp = *x & ~(3 << p);
    *x = tmp | (n << p);

}

void vector_length(struct vector *a){
    // Do strukture vector dostopamo preko kazalca
    a->length = sqrt((a->x)*(a->x) + (a->y)*(a->y));
}
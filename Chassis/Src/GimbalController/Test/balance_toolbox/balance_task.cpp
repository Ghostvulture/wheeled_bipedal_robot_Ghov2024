#include "balance_task.hpp"

void Quaternion_product_single_f32(const float32_t *qa, const float32_t *qb, float32_t *qr)
{
    qr[0] = qa[0] * qb[0] - qa[1] * qb[1] - qa[2] * qb[2] - qa[3] * qb[3];
    qr[1] = qa[0] * qb[1] + qa[1] * qb[0] + qa[2] * qb[3] - qa[3] * qb[2];
    qr[2] = qa[0] * qb[2] + qa[2] * qb[0] + qa[3] * qb[1] - qa[1] * qb[3];
    qr[3] = qa[0] * qb[3] + qa[3] * qb[0] + qa[1] * qb[2] - qa[2] * qb[1];
}

void Quaternion_product_f32(const float32_t *qa, const float32_t *qb, float32_t *qr, uint32_t nbQuaternions)
{
   uint32_t i;
   for(i=0; i < nbQuaternions; i++)
   {
     Quaternion_product_single_f32(qa, qb, qr);

     qa += 4;
     qb += 4;
     qr += 4;
   }
}



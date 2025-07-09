#include <Arduino.h>

struct EKF {
  float x[2];      // Durum vektörü [konum, hız]
  float P[2][2];   // Hata kovaryans matrisi
  float A[2][2];   // Durum geçiş matrisi
  float H[1][2];   // Ölçüm matrisi
  float Q[2][2];   // Proses gürültü kovaryansı
  float R[1][1];   // Ölçüm gürültü kovaryansı
  float dt;        // Zaman aralığı

  void init(float dt_val) {
    dt = dt_val;
    x[0] = 0.0;
    x[1] = 1.0;

    A[0][0] = 1; A[0][1] = dt;
    A[1][0] = 0; A[1][1] = 1;

    H[0][0] = 1; H[0][1] = 0;

    P[0][0] = 1; P[0][1] = 0;
    P[1][0] = 0; P[1][1] = 1;

    Q[0][0] = 0.1; Q[0][1] = 0;
    Q[1][0] = 0; Q[1][1] = 0.1;

    R[0][0] = 1;
  }

  void predict() {
    float x_new[2];
    x_new[0] = A[0][0]*x[0] + A[0][1]*x[1];
    x_new[1] = A[1][0]*x[0] + A[1][1]*x[1];
    x[0] = x_new[0];
    x[1] = x_new[1];

    float P_new[2][2];
    P_new[0][0] = A[0][0]*P[0][0] + A[0][1]*P[1][0];
    P_new[0][1] = A[0][0]*P[0][1] + A[0][1]*P[1][1];
    P_new[1][0] = A[1][0]*P[0][0] + A[1][1]*P[1][0];
    P_new[1][1] = A[1][0]*P[0][1] + A[1][1]*P[1][1];

    P[0][0] = P_new[0][0] + Q[0][0];
    P[0][1] = P_new[0][1] + Q[0][1];
    P[1][0] = P_new[1][0] + Q[1][0];
    P[1][1] = P_new[1][1] + Q[1][1];
  }

  void update(float z) {
    // Y = z - Hx
    float y = z - (H[0][0]*x[0] + H[0][1]*x[1]);

    // S = HPH' + R
    float S = H[0][0]*(P[0][0]*H[0][0] + P[0][1]*H[0][1]) + H[0][1]*(P[1][0]*H[0][0] + P[1][1]*H[0][1]) + R[0][0];

    // K = PH'/S
    float K[2];
    K[0] = (P[0][0]*H[0][0] + P[0][1]*H[0][1]) / S;
    K[1] = (P[1][0]*H[0][0] + P[1][1]*H[0][1]) / S;

    // x = x + Ky
    x[0] = x[0] + K[0]*y;
    x[1] = x[1] + K[1]*y;

    // P = (I-KH)P
    float P_new[2][2];
    P_new[0][0] = (1 - K[0]*H[0][0])*P[0][0] - K[0]*H[0][1]*P[1][0];
    P_new[0][1] = (1 - K[0]*H[0][0])*P[0][1] - K[0]*H[0][1]*P[1][1];
    P_new[1][0] = -K[1]*H[0][0]*P[0][0] + (1 - K[1]*H[0][1])*P[1][0];
    P_new[1][1] = -K[1]*H[0][0]*P[0][1] + (1 - K[1]*H[0][1])*P[1][1];

    P[0][0] = P_new[0][0];
    P[0][1] = P_new[0][1];
    P[1][0] = P_new[1][0];
    P[1][1] = P_new[1][1];
  }
};

EKF ekf;

void setup() {
  Serial.begin(9600);
  ekf.init(1.0);
}

void loop() {
  // Simüle edilmiş ölçüm (gürültülü gerçek konum)
  float true_position = millis() / 1000.0; // Zamanla artan pozisyon
  float measurement = true_position + random(-10, 10)*0.1;

  ekf.predict();
  ekf.update(measurement);

  Serial.print("Tahmin Edilen Konum: ");
  Serial.print(ekf.x[0]);
  Serial.print(", Tahmin Edilen Hız: ");
  Serial.println(ekf.x[1]);

  delay(1000);
}

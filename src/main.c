#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>
#include <math.h>

/* ===================== Configuração ===================== */
#define SAMPLE_PERIOD_MS   10      /* 100 Hz */
#define CALIB_SAMPLES      200     /* amostras para calibrar o bias (sensor parado) */
#define STILL_THRESHOLD    0.15f   /* m/s² : abaixo disso considera "parado" */
#define STILL_COUNT        20      /* amostras seguidas "parado" para zerar velocidade */

#define SAMPLER_STACK      2048
#define PRINTER_STACK      3072 
#define SAMPLER_PRIO       1
#define PRINTER_PRIO       5

/* ===================== Dispositivo ===================== */
static const struct device *const accel = DEVICE_DT_GET(DT_ALIAS(accel0));

/* ===================== Estado compartilhado ===================== */
struct motion_state {
	float a[3];   /* aceleração sem bias (m/s²) */
	float v[3];   /* velocidade (m/s)           */
	float p[3];   /* posição (m)                */
	float dt;     /* último dt (s)              */
};

static struct motion_state st;
static float bias[3];
K_MUTEX_DEFINE(st_lock);

/* ===================== Funções auxiliares ===================== */
static int read_accel(float out[3])
{
	struct sensor_value v[3];
	int ret;

	ret = sensor_sample_fetch(accel);
	if (ret) {
		return ret;
	}

	ret = sensor_channel_get(accel, SENSOR_CHAN_ACCEL_XYZ, v);
	if (ret) {
		return ret;
	}

	for (int i = 0; i < 3; i++) {
		out[i] = sensor_value_to_float(&v[i]);
	}
	return 0;
}

static void configure_odr(void)
{
	struct sensor_value odr = { .val1 = 100, .val2 = 0 };
	int ret = sensor_attr_set(accel, SENSOR_CHAN_ACCEL_XYZ,
				  SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);
	if (ret) {
		/* Nem todo driver suporta; o dt é medido, então continua funcionando */
		printk("Aviso: ODR nao configurado via driver (%d)\n", ret);
	}
}

/* Média de várias amostras com o sensor PARADO.
 * O bias inclui a gravidade na orientação inicial. */
static void calibrate(void)
{
	float sum[3] = { 0 };
	int n = 0;

	printk("Calibrando... mantenha o sensor parado.\n");

	while (n < CALIB_SAMPLES) {
		float a[3];

		if (read_accel(a) == 0) {
			for (int i = 0; i < 3; i++) {
				sum[i] += a[i];
			}
			n++;
		}
		k_msleep(SAMPLE_PERIOD_MS);
	}

	for (int i = 0; i < 3; i++) {
		bias[i] = sum[i] / CALIB_SAMPLES;
	}
	printk("Calibracao ok.\n");
}

static inline int32_t to_milli(float x)
{
	return (int32_t)(x * 1000.0f);
}

/* ===================== Thread: leitura + integração ===================== */
static void sampler(void *p1, void *p2, void *p3)
{
	if (!device_is_ready(accel)) {
		printk("Acelerometro nao esta pronto!\n");
		return;
	}

	configure_odr();
	calibrate();

	int64_t next_ms = k_uptime_get();
	uint32_t t_prev = k_cycle_get_32();
	float a_prev[3] = { 0 };
	int still = 0;

	while (1) {
		/* Período fixo, sem acumular o tempo gasto no I2C */
		next_ms += SAMPLE_PERIOD_MS;
		k_sleep(K_TIMEOUT_ABS_MS(next_ms));

		float a[3];

		if (read_accel(a) != 0) {
			continue;
		}

		/* dt real medido em ciclos (mais preciso que ms) */
		uint32_t now = k_cycle_get_32();
		float dt = (float)k_cyc_to_ns_floor64(now - t_prev) / 1e9f;
		t_prev = now;

		/* Remove bias (e gravidade da orientação inicial) */
		for (int i = 0; i < 3; i++) {
			a[i] -= bias[i];
		}

		/* Detecção de repouso (ZUPT): evita deriva da velocidade */
		bool quiet = true;

		for (int i = 0; i < 3; i++) {
			if (fabsf(a[i]) > STILL_THRESHOLD) {
				quiet = false;
			}
		}
		still = quiet ? still + 1 : 0;

		k_mutex_lock(&st_lock, K_FOREVER);

		if (still >= STILL_COUNT) {
			for (int i = 0; i < 3; i++) {
				st.v[i] = 0.0f;
				a[i] = 0.0f;
			}
		} else {
			for (int i = 0; i < 3; i++) {
				/* Integração trapezoidal, UMA vez por amostra */
				float v_old = st.v[i];

				st.v[i] += 0.5f * (a_prev[i] + a[i]) * dt;
				st.p[i] += 0.5f * (v_old + st.v[i]) * dt;
			}
		}

		for (int i = 0; i < 3; i++) {
			st.a[i] = a[i];
			a_prev[i] = a[i];
		}
		st.dt = dt;

		k_mutex_unlock(&st_lock);
	}
}

/* ===================== Thread: impressão ===================== */
static void printer(void *p1, void *p2, void *p3)
{
	while (1) {
		struct motion_state s;

		k_mutex_lock(&st_lock, K_FOREVER);
		s = st;
		k_mutex_unlock(&st_lock);

		/* ax,ay,az,vx,vy,vz,px,py,pz */
		printk("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
		       (double)s.a[0], (double)s.a[1], (double)s.a[2],
		       (double)s.v[0], (double)s.v[1], (double)s.v[2],
		       (double)s.p[0], (double)s.p[1], (double)s.p[2]);

		k_msleep(100);
	}
}

K_THREAD_DEFINE(sampler_tid, SAMPLER_STACK, sampler, NULL, NULL, NULL,
		SAMPLER_PRIO, 0, 0);
K_THREAD_DEFINE(printer_tid, PRINTER_STACK, printer, NULL, NULL, NULL,
		PRINTER_PRIO, 0, 0);

int main(void)
{
	return 0;
}
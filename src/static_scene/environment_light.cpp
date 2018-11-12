#include <iostream>
#include "environment_light.h"

using namespace std;
namespace CMU462 {
namespace StaticScene {

EnvironmentLight::EnvironmentLight(const HDRImageBuffer* envMap)
    : envMap(envMap) {
  // TODO: (PathTracer) initialize things here as needed
	int width = envMap->w;
	int height = envMap->h;
	sampleToWorld[0] = Vector3D(0, 0, -1);
	sampleToWorld[1] = Vector3D(1, 0, 0);
	sampleToWorld[2] = Vector3D(0, 1, 0);
	worldToSample = sampleToWorld.T();

	// compute pdf, p_theta before norm
	my_pdf.resize(height);
	my_cpdf.resize(height);
	my_p_theta.resize(height, 0);
	my_c_p_theta.resize(height, 0);
	int cnt = 0;
	double sum = 0.0;
	for (int i = 0; i < height; i++) {
		double v = (i + 0.5) / height;
		my_pdf[i].resize(width, 0);
		my_cpdf[i].resize(width, 0);
		for (int j = 0; j < width; j++) {
			double theta = v * PI;
			my_pdf[i][j] = envMap->data[cnt].illum() * sin(theta);
			cnt++;
			sum += my_pdf[i][j];
			my_p_theta[i] += my_pdf[i][j];
		}
	}
	// norm pdf sum to 1, compute p_theta and cpdf 
	double my_c_p_theta_sum = 0;
	for (int i = 0; i < height; i++) {
		my_p_theta[i] /= sum;
		my_c_p_theta_sum += my_p_theta[i];
		my_c_p_theta[i] = my_c_p_theta_sum;

		double my_cpdf_sum = 0;
		for (int j = 0; j < width; j++) {
			my_pdf[i][j] /= sum;
			if (my_p_theta[i] > 0) {
				my_cpdf_sum += my_pdf[i][j] / my_p_theta[i];
			}
			my_cpdf[i][j] = my_cpdf_sum;
		}
	}
}

Spectrum EnvironmentLight::sample_L(const Vector3D& p, Vector3D* wi,
                                    float* distToLight, float* pdf) const {
	// TODO: (PathTracer) Implement


	//// below is uniform sampling
	//double Xi1 = (double)(std::rand()) / RAND_MAX;
	//double Xi2 = (double)(std::rand()) / RAND_MAX;

	//double theta = acos(1 - 2 * Xi1);
	//double phi = 2.0 * PI * Xi2;

	//double xs = sinf(theta) * cosf(phi);
	//double ys = sinf(theta) * sinf(phi);
	//double zs = cosf(theta);

	//*distToLight = INF_D;
	//*pdf = 1.f / (4 * PI);
	//*wi = sampleToWorld * Vector3D(xs, ys, zs);

	//return sample_dir(Ray(p, *wi));


	// below is importance sampling
	int width = envMap->w;
	int height = envMap->h;
	double Xi1 = (double)(std::rand()) / RAND_MAX;
	double Xi2 = (double)(std::rand()) / RAND_MAX;
	vector<double>::const_iterator iter1 = lower_bound(my_c_p_theta.begin(), my_c_p_theta.end(), Xi1);
	int i = iter1 - my_c_p_theta.begin();
	i = min(i, height - 1);
	vector<double>::const_iterator iter2 = lower_bound(my_cpdf[i].begin(), my_cpdf[i].end(), Xi2);
	int j = iter2 - my_cpdf[i].begin();
	j = min(j, width - 1);

	double u = (j + 0.5) / width;
	double v = (i + 0.5) / height;
	double theta = v * PI;
	double phi = u * 2 * PI;

	double xs = sinf(theta) * cosf(phi);
	double ys = cosf(theta);
	double zs = sinf(theta) * sinf(phi);

	*distToLight = INF_D;
	*pdf = my_pdf[i][j] / ((PI/height)*(2*PI/width));
	*wi = sampleToWorld * Vector3D(xs, ys, zs);
	return sample_dir(Ray(p, *wi));
	
}

Spectrum EnvironmentLight::sample_dir(const Ray& r) const {
	// TODO: (PathTracer) Implement
	const Vector3D& dir = (worldToSample * r.d).unit();

	double theta = acos(dir.z);
	double phi = atan2(dir.y, dir.x) + PI;
	double u = phi / (2.0*PI);
	double v = theta / PI;

	int width = envMap->w;
	int height = envMap->h;

	// bilinear interpolation
	float x = u * width - 0.5;
	float y = v * height - 0.5;
	int sx = int(floor(x));
	int sy = int(floor(y));
	float u_ratio = x - sx;
	float v_ratio = y - sy;
	float u_opposite = 1 - u_ratio;
	float v_opposite = 1 - v_ratio;
	int sx0, sx1, sy0, sy1;
	sx0 = max(0, sx);
	sy0 = max(0, sy);
	sx1 = min(width - 1, sx + 1);
	sy1 = min(height - 1, sy + 1);

	Spectrum data1 = envMap->data[sx0 + sy0 * width];
	Spectrum data2 = envMap->data[sx1 + sy0 * width];
	Spectrum data3 = envMap->data[sx0 + sy1 * width];
	Spectrum data4 = envMap->data[sx1 + sy1 * width];

	return (data1*u_opposite + data2 * u_ratio)*v_opposite + (data3*u_opposite + data4 * u_ratio)*v_ratio;

}

}  // namespace StaticScene
}  // namespace CMU462

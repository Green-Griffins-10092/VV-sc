package org.firstinspires.ftc.griffins.Mahalanobis;

import com.qualcomm.robotcore.hardware.ColorSensor;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.apache.commons.math3.stat.correlation.Covariance;

import java.util.ArrayList;
import java.util.Arrays;

import static java.lang.Math.random;

/**
 * Created by David on 2/27/2017.
 * <p>
 * Implements a Mahalanobis Distance calculator, and a method to find the covariance and mean from a given set of data
 * See https://en.wikipedia.org/wiki/Mahalanobis_distance
 */

public class ColorSensorDistance {

    private RealVector[] mean;
    private RealMatrix[] covariance;
    private RealMatrix[] covarianceInverse;

    public ColorSensorDistance(RealVector[] mean, RealMatrix[] covariance) {
        if (mean.length != covariance.length && mean.length != ColorLabels.NUMBER_OF_LABELS) {
            throw new IllegalArgumentException("The number of means and covariances must both be " + ColorLabels.NUMBER_OF_LABELS);
        }

        this.mean = mean;
        this.covariance = covariance;

        covarianceInverse = new RealMatrix[covariance.length];

        for (int i = 0; i < covariance.length; i++) {
            covarianceInverse[i] = new LUDecomposition(covariance[i]).getSolver().getInverse();
        }
    }

    public static ColorSensorDistance getColorSensorDistanceFromData(ColorTrainingData[] data) {

        //create and initialize the array of sorted data
        ArrayList<ColorTrainingData>[] sortedData = new ArrayList[ColorLabels.NUMBER_OF_LABELS];

        for (int i = 0; i < sortedData.length; i++) {
            sortedData[i] = new ArrayList<>();
        }

        //sort the data into the array
        for (int i = 0; i < data.length; i++) {
            sortedData[data[i].label.index].add(data[i]);
        }

        //create and initialize the mean and covariance arrays
        RealVector[] mean = new RealVector[ColorLabels.NUMBER_OF_LABELS];
        RealMatrix[] covariance = new RealMatrix[ColorLabels.NUMBER_OF_LABELS];

        for (int i = 0; i < ColorLabels.NUMBER_OF_LABELS; i++) {
            mean[i] = new ArrayRealVector(4, 0);
            covariance[i] = new Array2DRowRealMatrix(4, 4);
        }

        //calculate the means
        for (int i = 0; i < sortedData.length; i++) {
            for (int j = 0; j < sortedData[i].size(); j++) {
                mean[i] = mean[i].add(sortedData[i].get(j)); //add it all together
            }

            //divide by the amount of data
            mean[i].mapMultiplyToSelf(1.0 / sortedData[i].size());
        }


        // use covariance class to get the covariance matrix instead
        for (int i = 0; i < sortedData.length; i++) {
            double[][] covarianceData = new double[sortedData[i].size()][sortedData[i].get(0).getDimension()];

            for (int j = 0; j < sortedData[i].size(); j++) {
                covarianceData[j] = sortedData[i].get(j).toArray();
            }

            Covariance covarianceCalculator = new Covariance(covarianceData, false);
            covariance[i] = covarianceCalculator.getCovarianceMatrix();
        }



        /*
        //calculate the covariance matrix - covariance_{i,j}= \sum_{k=0}^{k<n} [(data[k]_i-mean_i)(data[k]_j-mean_j)]/n
        for (int a = 0; a < sortedData.length; a++) {
            for (int i = 0; i < covariance[a].getRowDimension(); i++) {
                for (int j = 0; j < covariance[a].getColumnDimension(); j++) {

                    float sum = 0;
                    for (int k = 0; k < sortedData[a].size(); k++) {
                        sum += (sortedData[a].get(k).get(i) - mean[a].get(i)) * (sortedData[a].get(k).get(j) - mean[a].get(j));
                    }

                    covariance[a].setEntry(i, j, sum / (sortedData[a].size());

                }
            }
        }*/


        return new ColorSensorDistance(mean, covariance);
    }

    public static void main(String[] args) {
        ColorTrainingData[] trainingData = new ColorTrainingData[300];


        for (int i = 0; i < 100; i++) {
            trainingData[i] = new ColorTrainingData((int) (random() * 5), (int) (random() * 4), (int) (random() * 4), (int) (random() * 4), ColorLabels.UNDEFINED);
        }

        for (int i = 100; i < 200; i++) {
            trainingData[i] = new ColorTrainingData((int) (random() * 4) + 13, (int) (random() * 6) + 15, (int) (random() * 4) + 1, (int) (random() * 3) + 1, ColorLabels.RED);
        }

        for (int i = 200; i < 300; i++) {
            trainingData[i] = new ColorTrainingData((int) (random() * 4) + 13, (int) (random() * 3) + 1, (int) (random() * 6) + 14, (int) (random() * 6) + 15, ColorLabels.BLUE);
        }

        long time = System.currentTimeMillis();
        ColorSensorDistance tester = getColorSensorDistanceFromData(trainingData);
        System.out.println(System.currentTimeMillis() - time + "ms to compute");

        System.out.println(tester.mean[0]);
        System.out.println(tester.mean[1]);
        System.out.println(tester.mean[2]);
        System.out.println(tester.covariance[0]);
        System.out.println(tester.covariance[1]);
        System.out.println(tester.covariance[2]);
        System.out.println(tester.covarianceInverse[0]);
        System.out.println(tester.covarianceInverse[1]);
        System.out.println(tester.covarianceInverse[2]);

        ColorData data = new ColorData(15, 10, 3, 6);

        System.out.println(Arrays.toString(ColorLabels.values()));
        System.out.println(Arrays.toString(tester.calculateDistances(data)));

        System.out.println(tester.findColor(data));

    }

    public ColorLabels findColor(ColorData colorData) {
        double[] distances = calculateDistances(colorData);

        int indexOfMin = 0;
        double min = distances[0];
        for (int i = 1; i < distances.length; i++) {
            if (min >= distances[i]) {
                min = distances[i];
                indexOfMin = i;
            }
        }

        return ColorLabels.findByIndex(indexOfMin);
    }

    public double[] calculateDistances(ColorData colorData) {
        double[] distances = new double[ColorLabels.NUMBER_OF_LABELS];

        for (int i = 0; i < ColorLabels.NUMBER_OF_LABELS; i++) {
            RealVector dataMinusMean = colorData.subtract(mean[i]); //(x-m)

            //(x-m)^{T}*C^{-1}*(x-m)
            distances[i] = dataMinusMean.dotProduct(covarianceInverse[i].operate(dataMinusMean));
        }

        return distances;
    }


    public static enum ColorLabels {
        BLUE(0),
        RED(1),
        UNDEFINED(2);

        public static final int NUMBER_OF_LABELS = 3;
        private final int index;

        ColorLabels(int index) {
            this.index = index;
        }

        public static ColorLabels findByIndex(int index) {
            switch (index) {
                case 0:
                    return BLUE;
                case 1:
                    return RED;
                case 2:
                    return UNDEFINED;
                default:
                    throw new IllegalArgumentException("the index specified does not exist");
            }
        }
    }

    public static class ColorData extends ArrayRealVector {
        public ColorData(ColorSensor sensor) {
            this(sensor.alpha(), sensor.red(), sensor.green(), sensor.blue());
        }

        public ColorData(int alpha, int red, int green, int blue) {
            super(4);
            this.setEntry(0, alpha);
            this.setEntry(1, red);
            this.setEntry(2, green);
            this.setEntry(3, blue);
        }

        public int getAlpha() {
            return (int) this.getEntry(0);
        }

        public int getRed() {
            return (int) this.getEntry(1);
        }

        public int getGreen() {
            return (int) this.getEntry(2);
        }

        public int getBlue() {
            return (int) this.getEntry(3);
        }
    }

    public static class ColorTrainingData extends ColorData {
        private ColorLabels label;

        public ColorTrainingData(ColorSensor sensor, ColorLabels label) {
            super(sensor);
            this.label = label;
        }

        public ColorTrainingData(int alpha, int red, int green, int blue, ColorLabels label) {
            super(alpha, red, green, blue);
            this.label = label;
        }

        public ColorLabels getLabel() {
            return label;
        }
    }

}

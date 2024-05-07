package com.example.magandorid;

import java.util.ArrayList;

public class JniCallNative {
    static {
        System.loadLibrary("mynativeso");
    }
    private static native double[] solve_mag(double[] readings,
                                              double[] PSensor,
                                              double[] init_param);
//    public static native  int  getAddFromNative(int a,int b);

//    public static void main(String[] args) {


//    }
}

//    ArrayList<Double> init_param = new ArrayList<>();
//    ArrayList<Double> readings = new ArrayList<>();
//    ArrayList<Double> PSensor = new ArrayList<>();
//    Double[] temp = new Double[]{8.350, 2.8024, 5.7614, 8.350, 0.5418, 8.3268, 8.350, 0.0, 0.0, 8.350, -2.0236, 5.7614, -8.350, -2.0236, 5.7614,
//            -8.350, 0.5418, 8.3268, -8.350, 2.8024, 5.7614, -8.350, 0.0, 0.0};
//        for (int i = 0; i < temp.length; i++) {
//        PSensor.add(temp[i]);
//        }
////        (18.150001525878906, 38.70000076293945, 61.46799850463867)	(-26.250001907348633, 6.450000286102295, 8.711999893188477)	(-14.700000762939453, 5.700000286102295, 45.97999954223633)	(-20.55000114440918, -2.700000047683716, 21.053998947143555)	(-10.800000190734863, -5.850000381469727, -0.7260000109672546)	(-19.05000114440918, -12.90000057220459, 8.470000267028809)	(13.350000381469727, 28.950000762939453, 73.80999755859375)	(-35.400001525878906, -19.80000114440918, 17.423999786376953)
//
//        readings.add(18.4);
//        readings.add(38.4);
//        readings.add(28.4);
//        readings.add(58.4);
//        readings.add(28.4);
//        readings.add(18.4);
//        readings.add(48.4);
//        readings.add(58.4);
//        readings.add(78.4);
//        readings.add(58.4);
//        readings.add(48.4);
//        readings.add(28.4);
//        readings.add(28.4);
//        readings.add(18.4);
//        readings.add(58.4);
//        readings.add(78.4);
//        readings.add(58.4);
//        readings.add(78.4);
//        readings.add(88.4);
//        readings.add(88.4);
//        readings.add(128.4);
//        readings.add(28.4);
//        readings.add(68.4);
//        readings.add(18.4);
//
//
////    [8.350, 0.5418, 8.3268],
////    [8.350, 0, 0],
////    [8.350, -2.0236, 5.7614],
////    [-8.350, -2.0236, 5.7614],
////    [-8.350, 0.5418, 8.3268],
////    [-8.350, 2.8024, 5.7614],
////    [-8.350, 0, 0],
//
//        init_param.add(40 / Math.sqrt(2) * 1e-6);
//        init_param.add(40 / Math.sqrt(2) * 1e-6);
//        init_param.add((double) 0);
//        init_param.add(Math.log(2));
//        init_param.add(1e-2 * (5));
//        init_param.add(1e-2 * (10));
//        init_param.add(1e-2 * (2));
//        init_param.add(Math.PI);
//        init_param.add(Math.PI);
//        Double[] readings1 = new Double[24];
//        Double[] PSensor1 = new Double[24];
//        Double[] init_param1 = new Double[9];
//        for (int i = 0; i < readings.size(); i++) {
//        readings1[i] = readings.get(i);
//        }
//        for (int i = 0; i < PSensor.size(); i++) {
//        PSensor1[i] = PSensor.get(i);
//        }
//        for (int i = 0; i < init_param.size(); i++) {
//        init_param1[i] = init_param.get(i);
//        }
//        System.out.println(solve_1mag(readings1, PSensor1, init_param1));


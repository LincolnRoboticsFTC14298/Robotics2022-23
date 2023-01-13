package org.firstinspires.ftc.teamcode.filters.kalmanFilter

//import javafx.application.Application
//import javafx.scene.Scene
//import javafx.scene.chart.LineChart
//import javafx.scene.chart.NumberAxis
//import javafx.scene.chart.XYChart
//import javafx.scene.layout.FlowPane
//import javafx.stage.Stage
//import org.ejml.simple.SimpleMatrix
//import org.firstinspires.ftc.teamcode.filters.MeasurementModel
//import java.util.*
//import kotlin.jvm.JvmStatic
//import kotlin.math.abs

class KalmanSimulation {//: Application() {
//    var timeStep = 0.01
//    override fun start(stage: Stage) {
//        val timeData = time(100.0, timeStep)
//        val realState = groundTruth(timeData);
//        val measuredPosition = interjectNoise(realState[0], 0.1)
//        val predictedState = kalmanFilter(measuredPosition)
//
//        stage.title = "EKF Simulation"
//
//        val positionError = difference(realState[0], predictedState[0])
//        val positionChart = generateGraph(arrayOf(makeData(timeData, positionError)), "Position Error", "Time", "Error")
//
//        val velocityError = difference(realState[1], predictedState[1])
//        val velocityChart = generateGraph(arrayOf(makeData(timeData, velocityError)), "Velocity Error", "Time", "Error")
//
//        val accelerationError = difference(realState[2], predictedState[2])
//        val accelerationChart = generateGraph(arrayOf(makeData(timeData, accelerationError)), "Acceleration Error", "Time", "Error")
//
//
////        val differenceMeasurement = XYChart.Series<Number, Number>()
////        val dPM = difference(realState[0], measuredPosition)
////        differenceMeasurement.name = "Difference in Measured Position"
////        differenceMeasurement.data.addAll(makeData(timeData, dPM))
//
//        val root = FlowPane()
//        root.children.addAll(positionChart, velocityChart, accelerationChart)
//        val scene = Scene(root, 1500.0, 900.0)
//        stage.scene = scene
//        stage.show()
//    }
//
//    fun generateGraph(data: Array<Array<XYChart.Data<Number, Number>>>, title: String, xAxisTitle: String, yAxisTitle: String) : LineChart<Number, Number> {
//        val xAxis = NumberAxis()
//        val yAxis = NumberAxis()
//        xAxis.label = xAxisTitle
//        yAxis.label = yAxisTitle
//
//        val chart = LineChart(xAxis, yAxis)
//        chart.createSymbols = false
//        chart.title = title
//
//        for (d in data) {
//            val series = XYChart.Series<Number, Number>()
//            series.data.addAll(d)
//            chart.data.add(series)
//        }
//
//        return chart
//    }
//
//    fun time(duration: Double, timestep: Double): DoubleArray {
//        val size = (duration / timestep).toInt()
//        val t = DoubleArray(size)
//        for (i in 0 until size) {
//            t[i] = timestep * i
//        }
//        return t
//    }
//
//    fun groundTruth(t: Double): DoubleArray {
//        return doubleArrayOf(t * t, 2.0 * t, 2.0)
//    }
//
//    fun groundTruth(time: DoubleArray): Array<DoubleArray> {
//        val pos = DoubleArray(time.size)
//        val vel = DoubleArray(time.size)
//        val acc = DoubleArray(time.size)
//        for (i in time.indices) {
//            val state = groundTruth(time[i])
//            pos[i] = state[0]
//            vel[i] = state[1]
//            acc[i] = state[2]
//        }
//        return arrayOf(pos, vel, acc)
//    }
//
//    private val r = Random()
//    fun interjectNoise(series: DoubleArray, sd: Double): DoubleArray {
//        val newSeries = DoubleArray(series.size)
//        for (i in series.indices) {
//            newSeries[i] = series[i] + r.nextGaussian() * sd
//        }
//        return newSeries
//    }
//
//    fun difference(series1: DoubleArray, series2: DoubleArray): DoubleArray {
//        val diff = DoubleArray(series1.size)
//        for (i in series1.indices) {
//            diff[i] = series1[i] - series2[i]
//        }
//        return diff
//    }
//
//    fun absdiff(series1: DoubleArray, series2: DoubleArray): DoubleArray {
//        val diff = DoubleArray(series1.size)
//        for (i in series1.indices) {
//            diff[i] = abs(series1[i] - series2[i])
//        }
//        return diff
//    }
//
//    fun makeData(x: DoubleArray, y: DoubleArray): Array<XYChart.Data<Number, Number>> {
//        return Array(x.size) { i -> XYChart.Data(x[i], y[i]) }
//    }
//
//    private val filter = KalmanFilter(
//        OneDModel(),
//        OneDSensor(),
//        SimpleMatrix(arrayOf(doubleArrayOf(0.0), doubleArrayOf(2.0), doubleArrayOf(0.1))),
//        SimpleMatrix.diag(1.0, 1.0, 1.0)
//    )
//
//    fun kalmanFilter(measurements: DoubleArray): Array<DoubleArray> {
//        val xhat = DoubleArray(measurements.size)
//        val vhat = DoubleArray(measurements.size)
//        val ahat = DoubleArray(measurements.size)
//        for (i in measurements.indices) {
//            filter.predict(null, timeStep)
//            filter.update(SimpleMatrix(arrayOf(doubleArrayOf(measurements[i]))))
//            val state = filter.stateEstimate.ddrm.data
//            xhat[i] = state[0]
//            vhat[i] = state[1]
//            ahat[i] = state[2]
//        }
//        return arrayOf(xhat, vhat, ahat)
//    }
//
//    internal class OneDModel : KalmanProcessModel {
//        override fun predictState(
//            previousState: SimpleMatrix,
//            u: SimpleMatrix?,
//            dt: Double
//        ): SimpleMatrix {
//            return getStateTransitionMatrix(previousState, u, dt).mult(previousState)
//        }
//
//        override fun getStateTransitionMatrix(
//            previousState: SimpleMatrix,
//            u: SimpleMatrix?,
//            dt: Double
//        ): SimpleMatrix {
//            return SimpleMatrix(
//                arrayOf(
//                    doubleArrayOf(1.0, dt, 0.5 * dt * dt),
//                    doubleArrayOf(0.0, 1.0, dt),
//                    doubleArrayOf(0.0, 0.0, 1.0)
//                )
//            )
//        }
//
//        override fun getProcessNoise(dt: Double): SimpleMatrix {
//            val dt4 = dt * dt * dt * dt
//            val dt3 = dt * dt * dt
//            val dt2 = dt * dt
//            return SimpleMatrix(
//                arrayOf(
//                    doubleArrayOf(dt4 / 4.0, dt3 / 2.0, dt2 / 2.0),
//                    doubleArrayOf(dt3 / 2.0, dt2, dt),
//                    doubleArrayOf(dt2 / 2.0, dt, 1.0)
//                )
//            ).scale(0.1)
//        }
//    }
//
//    internal class OneDSensor : KalmanMeasurementModel {
//        override fun predictObservation(state: SimpleMatrix): SimpleMatrix {
//            return getObservationMatrix(state).mult(state)
//        }
//
//        override fun getObservationMatrix(state: SimpleMatrix): SimpleMatrix {
//            return SimpleMatrix(arrayOf(doubleArrayOf(1.0, 0.0, 0.0)))
//        }
//
//        override fun getObservationNoise(): SimpleMatrix {
//            return SimpleMatrix.diag(1e5)
//        }
//    }
//
//    companion object {
//        @JvmStatic
//        fun main(args: Array<String>) {
//            launch(KalmanSimulation::class.java, *args)
//        }
//    }
}
package com.example.magandorid

//import android.os.Bundle

//import androidx.appcompat.app.AppCompatActivity

import android.Manifest
import android.annotation.SuppressLint
import android.bluetooth.BluetoothGatt
import android.bluetooth.BluetoothGattCallback
import android.bluetooth.BluetoothGattCharacteristic
import android.bluetooth.BluetoothGattDescriptor
import android.bluetooth.BluetoothManager
import android.bluetooth.BluetoothProfile
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import android.os.Environment
import android.os.Handler
import android.os.Looper
import android.util.Log
import android.view.View
import android.widget.Button
import android.widget.ProgressBar
import android.widget.ScrollView
import android.widget.TextView
import android.widget.Toast
import androidx.annotation.RequiresApi
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import androidx.recyclerview.widget.LinearLayoutManager
import androidx.recyclerview.widget.RecyclerView
import com.example.magandorid.blescanner.BleScanManager
import com.example.magandorid.blescanner.adapter.BleDeviceAdapter
import com.example.magandorid.blescanner.model.BleDevice
import com.example.magandorid.blescanner.model.BleScanCallback
import com.lorenzofelletti.permissions.PermissionManager
import com.lorenzofelletti.permissions.dispatcher.dsl.checkPermissions
import com.lorenzofelletti.permissions.dispatcher.dsl.doOnDenied
import com.lorenzofelletti.permissions.dispatcher.dsl.doOnGranted
import com.lorenzofelletti.permissions.dispatcher.dsl.showRationaleDialog
import com.lorenzofelletti.permissions.dispatcher.dsl.withRequestCode
import java.io.BufferedReader
import java.io.File
import java.io.FileReader
import java.io.IOException
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.time.LocalDateTime
import java.time.format.DateTimeFormatter
import java.util.UUID
import kotlin.math.ln
import kotlin.math.sqrt
import org.ejml.simple.SimpleMatrix
import kotlin.math.pow

import android.view.LayoutInflater
//import android.view.View
import android.view.ViewGroup
//import android.widget.TextView
//import androidx.recyclerview.widget.RecyclerView
import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.util.AttributeSet
import kotlinx.coroutines.delay
import java.util.concurrent.CopyOnWriteArrayList
//import android.view.View
import kotlin.random.Random

fun generateRandomInt(range: IntRange): Int {
    return Random.nextInt(range.first, range.last + 5)
}

class PointGenerator {
//    private var index = 0.0

    fun getNextPoint(): DoubleArray {
        val x = generateRandomInt(1..100)
        val y = Math.sin(x.toDouble()) // Generate y as a sinusoidal function of x
        val z = Math.cos(x.toDouble()) // Generate z as a cosine function of x
//        index += 0.1 // Increment the index for the next point
        return doubleArrayOf(x.toDouble(), y, z)
    }
}

class PlotView @JvmOverloads constructor(
    context: Context, attrs: AttributeSet? = null, defStyleAttr: Int = 0
) : View(context, attrs, defStyleAttr) {
    private var points = CopyOnWriteArrayList<Triple<Float, Float, Float>>()
    private val maxPoints = 40
    private val paint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.FILL_AND_STROKE
        strokeWidth = 5f
        color = android.graphics.Color.RED
    }

    private var centerX = 0f
    private var centerY = 0f
    private var scaleFactorX = 1f
    private var scaleFactorY = 1f
    private var maxX = 1f
    private var maxY = 1f
    private val defaultMaxX = 0.3f
    private val defaultMaxY = 0.3f
    private var lastUpdateTime = System.currentTimeMillis()
    private val resetDelay = 5000 // 5 seconds delay to reset boundaries

    init {
        addOnLayoutChangeListener { _, _, _, _, _, _, _, _, _ ->
            centerX = width / 3f
            centerY = height / 3f
        }
        // 初始化默认边界值
        maxX = defaultMaxX
        maxY = defaultMaxY
    }

    @Synchronized
    fun addPoint(x: Float, y: Float, pointSize: Float) {
        if (points.size >= maxPoints) {
            points.removeAt(0)
        }
        points.add(Triple(x, y, pointSize))
        checkAndUpdateBoundaries(x, y)
        invalidate()
    }

    private fun checkAndUpdateBoundaries(x: Float, y: Float) {
        val absX = kotlin.math.abs(x)
        val absY = kotlin.math.abs(y)
        var boundariesUpdated = false

        if (absX > maxX) {
            maxX = absX
            boundariesUpdated = true
        }
        if (absY > maxY) {
            maxY = absY
            boundariesUpdated = true
        }

        if (boundariesUpdated) {
            updateScaleFactors()
            lastUpdateTime = System.currentTimeMillis()
        } else if (System.currentTimeMillis() - lastUpdateTime > resetDelay) {
            // Check if we need to reset boundaries to default values
            resetBoundaries()
            updateScaleFactors()
        }
    }

    private fun resetBoundaries() {
        maxX = defaultMaxX
        maxY = defaultMaxY
    }

    private fun updateScaleFactors() {
        scaleFactorX = if (maxX != 0f) (width / 2f) / maxX else 1f
        scaleFactorY = if (maxY != 0f) (height / 2f) / maxY else 1f
    }

    override fun onDraw(canvas: Canvas?) {
        super.onDraw(canvas)
        Log.w("points size", "size ${points.size}")

        var lastPoint: Pair<Float, Float>? = null
        points.forEach { point ->
            val drawX = centerX + point.first * scaleFactorX
            val drawY = centerY - point.second * scaleFactorY
            paint.strokeWidth = point.third
            canvas?.drawPoint(drawX, drawY, paint)

            //You can use the following code to draw the line

//            if (lastPoint != null) {
//                canvas?.drawLine(lastPoint!!.first, lastPoint!!.second, drawX, drawY, paint)
//            }
//            lastPoint = Pair(drawX, drawY)
        }
    }

    @Synchronized
    fun clearPoints() {
        points.clear()
        invalidate()
    }
}



class ResultsAdapter : RecyclerView.Adapter<ResultsAdapter.ViewHolder>() {
    private val resultsList = mutableListOf<String>()

    // Inner class that defines each item in RecyclerView
    class ViewHolder(view: View) : RecyclerView.ViewHolder(view) {
        val resultTextView: TextView = view.findViewById(android.R.id.text1)
    }

    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): ViewHolder {
        // Inflate a simple layout for each item
        val view = LayoutInflater.from(parent.context).inflate(
            android.R.layout.simple_list_item_1, parent, false
        )
        return ViewHolder(view)
    }

    override fun onBindViewHolder(holder: ViewHolder, position: Int) {
        holder.resultTextView.text = resultsList[position]
    }

    override fun getItemCount(): Int = resultsList.size

    // Method to add new results dynamically
    fun addResult(newResult: String) {
        resultsList.add(newResult)
        notifyItemInserted(resultsList.size - 1)
    }

    // Method to clear all results
    fun clearResults() {
        resultsList.clear()
        notifyDataSetChanged()
    }
}

class Calibration(val nSensor: Int, val readings: MutableList<DoubleArray>) {

    fun caliResult(): Pair<Array<DoubleArray>, Array<DoubleArray>> {
        val data = SimpleMatrix(readings.toTypedArray())

        val offX = DoubleArray(nSensor)
        val offY = DoubleArray(nSensor)
        val offZ = DoubleArray(nSensor)
        val scaleX = DoubleArray(nSensor)
        val scaleY = DoubleArray(nSensor)
        val scaleZ = DoubleArray(nSensor)

        for (i in 0 until nSensor) {
            val mag = data.cols(i * 3, i * 3 + 3)
            val ones = SimpleMatrix(mag.numRows(), 1)
            for (r in 0 until ones.numRows()) {
                ones[r, 0] = 1.0  // Fill the matrix with 1s
            }

            val H = SimpleMatrix(mag.numRows(), 6).apply {
                insertIntoThis(0, 0, mag)
                insertIntoThis(0, 3, mag.extractVector(false, 1).elementPower(2).scale(-1.0))
                insertIntoThis(0, 4, mag.extractVector(false, 2).elementPower(2).scale(-1.0))
                insertIntoThis(0, 5, ones)
            }
            val w = mag.extractVector(false, 0).elementPower(2)
            val tmp = (H.transpose().mult(H)).invert().mult(H.transpose())
            val X = tmp.mult(w)

            offX[i] = X[0] / 2
            offY[i] = X[1] / (2 * X[3])
            offZ[i] = X[2] / (2 * X[4])
            val temp = X[5] + offX[i].pow(2) + X[3] * offY[i].pow(2) + X[4] * offZ[i].pow(2)
            scaleX[i] = kotlin.math.sqrt(temp)
            scaleY[i] = kotlin.math.sqrt(temp / X[3])
            scaleZ[i] = kotlin.math.sqrt(temp / X[4])
        }

        val offset = Array(nSensor) { i -> doubleArrayOf(offX[i], offY[i], offZ[i]) }
        val scale = Array(nSensor) { i -> doubleArrayOf(scaleX[i], scaleY[i], scaleZ[i]) }

        return offset to scale
    }

    private fun SimpleMatrix.cols(startCol: Int, endCol: Int): SimpleMatrix {
        return this.extractMatrix(0, this.numRows(), startCol, endCol)
    }

    private fun SimpleMatrix.elementPower(power: Int): SimpleMatrix {
        val result = SimpleMatrix(this.numRows(), this.numCols())
        for (r in 0 until this.numRows()) {
            for (c in 0 until this.numCols()) {

                result[r, c] = this[r, c].pow(power.toDouble())
            }
        }
        return result
    }

}



class CalibrationActivity : AppCompatActivity() {
    //The position of the sensor in MagDot
    var psensor=doubleArrayOf(0.0, -0.008, 0.0,0.005, -0.005, 0.0,-0.005, -0.005, 0.0
        ,0.01, 0.0, 0.0,0.0, 0.0,0.0,-0.01, 0.0, 0.0,0.005, 0.005, 0.0,
        -0.005, 0.005, 0.0,0.01,0.01,0.0,-0.01,0.01,0.0)
    //The position of the sensor in MagX
//    var psensor=doubleArrayOf(3.0, -3.0, -1.63,-3.0, -3.0, -1.63,-3.0, 3.0, -1.63
//        ,3.0, 3.0, -1.63,3.0, 0.0, 1.63,0.0, -3.0, 1.63,-3.0, 0.0, 1.63,0.0, 3.0, 1.63)

    //The initial params of magnet
    var initparam = doubleArrayOf(40 / sqrt(2.0) * 1e-6,40 / sqrt(2.0) * 1e-6,0.0,
        ln(0.2),(-0.0001),(-0.05),(-0.0017),Math.PI,0.0)
    var initparam1 = doubleArrayOf(40 / sqrt(2.0) * 1e-6,40 / sqrt(2.0) * 1e-6,0.0,
        ln(0.2),(-0.0001),(-0.05),(-0.0017),Math.PI,0.0)


    val readingsList = mutableListOf<DoubleArray>()
    val CalibrationData = mutableListOf<DoubleArray>()

    var myparam:DoubleArray = DoubleArray(9)
    private lateinit var btnStartScan: Button
//    private lateinit var resultsRecyclerView: RecyclerView
    private lateinit var resultsAdapter: ResultsAdapter
    private lateinit var resultsTextView: TextView
    private lateinit var resultsScrollView: ScrollView
    private lateinit var plotView1: PlotView
    private lateinit var plotView2: PlotView
    private lateinit var btnStartCalibration: Button
//    private lateinit var progressBar: ProgressBar
//    private lateinit var statusText: TextView
    private lateinit var startButton: Button
    private val handler = Handler()
    private var progressStatus = 0
    private lateinit var calibrationOffset: Array<DoubleArray>
    private lateinit var calibrationScale: Array<DoubleArray>
    private var isCalibrated = false // To check if calibration has been performed

    //    private val startButton: Button? = null
//    private val progressStatus = 0
//    private val handler = Handler()
    private lateinit var permissionManager: PermissionManager
    private lateinit var btManager: BluetoothManager
    private lateinit var bleScanManager: BleScanManager
    private lateinit var foundDevices: MutableList<BleDevice>
    // Top level declaration
    private var bluetoothGatt: BluetoothGatt? = null
    private val GATT_MAX_MTU_SIZE = 517






    private val gattCallback = @SuppressLint("NewApi")
    object : BluetoothGattCallback() {
        @SuppressLint("MissingPermission")
        override fun onConnectionStateChange(gatt: BluetoothGatt, status: Int, newState: Int) {
            val deviceAddress = gatt.device.address

            if (status == BluetoothGatt.GATT_SUCCESS) {
                if (newState == BluetoothProfile.STATE_CONNECTED) {
                    Log.w("BluetoothGattCallback", "Successfully connected to $deviceAddress")
                    // TODO: Store a reference to BluetoothGatt
                    Handler(Looper.getMainLooper()).post {
                        gatt?.discoverServices()
                    }
                    //gatt?.discoverServices()
                } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                    Log.w("BluetoothGattCallback", "Successfully disconnected from $deviceAddress")
                    gatt.close()
                }
            } else {
                Log.w("BluetoothGattCallback", "Error $status encountered for $deviceAddress! Disconnecting...")
                gatt.close()
            }
        }


        // Read the MLX90393 sensor data
        // Version 1
        override fun onCharacteristicChanged(
            gatt: BluetoothGatt,
            characteristic: BluetoothGattCharacteristic
        ) {
            //
            val arrayOfByte: ByteArray = characteristic.value
            // Little endian conversion
            val buffer = ByteBuffer.wrap(arrayOfByte).order(ByteOrder.LITTLE_ENDIAN)

            var readings:DoubleArray = DoubleArray(30)

            val currentDateTime = LocalDateTime.now()

            // Format the time
            val formatter = DateTimeFormatter.ofPattern("yyyy-MM-dd HH:mm:ss")
            val formattedDateTime = currentDateTime.format(formatter)
//            val mixedArray: Array<Any> = arrayOf(formattedDateTime, readings)
//            readings[0]=formattedDateTime
            for (i in 0..9) {
                val value1 = buffer.getFloat(12 * i)
                val value2 = buffer.getFloat(12 * i + 4)
                val value3 = buffer.getFloat(12 * i + 8)
                var j = i + 1

                readings[3*i]=value1.toDouble()
                readings[3*i+1]=value2.toDouble()
                readings[3*i+2]=value3.toDouble()
//                    temp = readings.
//                    Log.w("readings", "Sensor ($readings.get(0),$readings.get(1),$readings.get(2))")
                Log.w("BluetoothGattCallback", "Sensor $j: ($value1, $value2, $value3)")
            }

            //judge if has calibrated
            if (isCalibrated) {
                // Calculate the mean of all scales to use as the normalization factor
                var totalScaleSum = 0.0
                var count = 0

                // Sum up all individual scale factors across each sensor's three axes
                for (scale in calibrationScale) {
                    Log.w("scale", "scale : ${scale.joinToString(", ")}")
                    totalScaleSum += scale.sum()
                    count += scale.size
                }

                // Calculate the mean scale
                val meanScale = totalScaleSum / 30

                // Apply calibration offset and scale, then normalize using the mean scale
                for (i in 0..9) {
                    readings[3 * i] = (readings[3 * i] - calibrationOffset[i][0]) / calibrationScale[i][0] * meanScale
                    readings[3 * i + 1] = (readings[3 * i + 1] - calibrationOffset[i][1]) / calibrationScale[i][1] * meanScale
                    readings[3 * i + 2] = (readings[3 * i + 2] - calibrationOffset[i][2]) / calibrationScale[i][2] * meanScale
                }
                val maxAbsValue = initparam.sliceArray(4 until 7).maxOf { kotlin.math.abs(it) }
                if (maxAbsValue > 1) {
                    initparam = initparam1.copyOf()
                }
                val result: DoubleArray = solve_1mag(readings, psensor, initparam)
                initparam = result
                val subsetArray = result.sliceArray(4..8)
                subsetArray[0] = subsetArray[0]*1e2
                subsetArray[1] = subsetArray[1]*1e2
                subsetArray[2] = subsetArray[2]*1e2
                updateResults(subsetArray)
                val newSubsetArray = doubleArrayOf(subsetArray[0], subsetArray[1], subsetArray[2])
//                val newSubsetArray = doubleArrayOf(subsetArray[0], subsetArray[1], subsetArray[2])

                updatePlot(newSubsetArray)  // Draw the tracking results

                Log.w("init results", "results ${initparam[0]},${initparam[1]},${initparam[2]},${initparam[3]},${initparam[4]},${initparam[5]}," +
                        "${initparam[6]},${initparam[7]},${initparam[8]}")
            } else {
                Log.w("BluetoothGattCallback", "No calibration data available.")
            }

        }


        external fun stringFromJNI(): String
        //        external fun solve_1mag(
//            readings1: DoubleArray?,
//            PSensor1: DoubleArray?,
//            init_param1: DoubleArray?
//        ): DoubleArray?
        override  fun onCharacteristicRead(
            gatt: BluetoothGatt,
            characteristic: BluetoothGattCharacteristic,
            byteArr: ByteArray,
            status: Int
        ) {
            Log.w("BluetoothGattCallback", "Characteristic ${characteristic.uuid} read | status: $status")
        }

        override fun onDescriptorRead(
            gatt: BluetoothGatt,
            descriptor: BluetoothGattDescriptor,
            status: Int,
            value: ByteArray
        ) {
            super.onDescriptorRead(gatt, descriptor, status, value)
            // log the received value
            Log.w("BluetoothGattCallback", "Descriptor ${descriptor.uuid} read | status: $status | value: ${String(value)}")
        }

        override fun onDescriptorWrite(
            gatt: BluetoothGatt?,
            descriptor: BluetoothGattDescriptor?,
            status: Int
        ) {
            if (status == BluetoothGatt.GATT_SUCCESS) {
                // Notifications enabled, ready to receive data
                Log.w("BluetoothGattCallback", "Notification enabled")
            } else {
                // Failed to enable notifications, handle error
                Log.w("BluetoothGattCallback", "Failed to enable notifications")
            }
        }

        override fun onMtuChanged(gatt: BluetoothGatt, mtu: Int, status: Int) {
            Log.w( "Large Bytes", "ATT MTU changed to $mtu, success: ${status == BluetoothGatt.GATT_SUCCESS}")
        }

        @RequiresApi(Build.VERSION_CODES.TIRAMISU)
        @SuppressLint("MissingPermission")
        override fun onServicesDiscovered(gatt: BluetoothGatt, status: Int) {
            with(gatt) {
                Log.w("BluetoothGattCallback", "Discovered ${services.size} services for ${device.address}")
                // log the service detail information
                services.forEach { service ->
                    Log.w("BluetoothGattCallback", "Service: ${service.uuid}")
                    service.characteristics.forEach { characteristic ->
                        Log.w("BluetoothGattCallback", "Characteristic: ${characteristic.uuid}")
                        characteristic.descriptors.forEach { descriptor ->
                            Log.w("BluetoothGattCallback", "Descriptor: ${descriptor.uuid}")
                        }
                    }
                }

                // Nordic UART Service
                val uartService =
                    gatt.getService(UUID.fromString("6e400001-b5a3-f393-e0a9-e50e24dcca9e"))
                // enable UART service

                gatt.requestMtu(GATT_MAX_MTU_SIZE)

                // RX characteristic for write
                val rxCharacteristic =
                    uartService?.getCharacteristic(UUID.fromString("6e400002-b5a3-f393-e0a9-e50e24dcca9e"))

                // TX characteristic for notify
                val txCharacteristic =
                    uartService?.getCharacteristic(UUID.fromString("6e400003-b5a3-f393-e0a9-e50e24dcca9e"))

                // gatt.setCharacteristicNotification(rxCharacteristic, true)
                gatt.setCharacteristicNotification(txCharacteristic, true)

                // Characteristic user descriptor
                val descriptor =
                    txCharacteristic?.getDescriptor(UUID.fromString("00002902-0000-1000-8000-00805f9b34fb"))


                // Enable notifications
                if (descriptor != null) {
                    if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
                        gatt.writeDescriptor(
                            descriptor,
                            BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
                        )
                        // gatt.readDescriptor(descriptor)
                    }
                }

                // Call site
            }
        }

    }


    @SuppressLint("MissingPermission", "NewApi")
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_calibration)

//        resultsRecyclerView = findViewById(R.id.resultsRecyclerView)
//        resultsRecyclerView.layoutManager = LinearLayoutManager(this)

        // Set up the adapter
        resultsAdapter = ResultsAdapter()
        resultsTextView = findViewById(R.id.resultsTextView)
//        resultsRecyclerView.adapter = resultsAdapter
        permissionManager = PermissionManager(this)
        permissionManager buildRequestResultsDispatcher {
            withRequestCode(BLE_PERMISSION_REQUEST_CODE) {
                checkPermissions(blePermissions)
                showRationaleDialog(getString(R.string.ble_permission_rationale))
                doOnGranted { bleScanManager.scanBleDevices() }
                doOnDenied {
                    Toast.makeText(
                        this@CalibrationActivity,
                        getString(R.string.ble_permissions_denied_message),
                        Toast.LENGTH_LONG
                    ).show()
                }
            }
        }

        // RecyclerView handling
        val rvFoundDevices = findViewById<View>(R.id.rv_found_devices) as RecyclerView
        foundDevices = BleDevice.createBleDevicesList()
        val adapter = BleDeviceAdapter(foundDevices)
        rvFoundDevices.adapter = adapter
        rvFoundDevices.layoutManager = LinearLayoutManager(this)

        // BleManager creation
        btManager = getSystemService(BluetoothManager::class.java)
        bleScanManager = BleScanManager(btManager, 500, scanCallback = BleScanCallback({
            // Adding MAC address to the list of found devices
            val address = it?.device?.address
            if (address.isNullOrBlank()) return@BleScanCallback
            val name = it?.device?.name ?: "Unknown device"

            Log.i("ScanCallback", "Found BLE device! Name: $name, address: $address")

            // set the device bluetooth address
            if (it?.device?.address == "DE:31:41:0F:0F:0D") {
                Log.w("ScanResultAdapter", "Connecting to $address")
                val gatt = it?.device?.connectGatt(this, false, gattCallback)
//                bluetoothGatt = it?.device?.connectGatt(this@CalibrationActivity, false, gattCallback)
            }



            val device = BleDevice(address)
            if (!foundDevices.contains(device)) {
//                if (DEBUG) {
//                    Log.d(
//                        BleScanCallback::class.java.simpleName,
//                        "${this.javaClass.enclosingMethod?.name} - Found device: $name"
//                    )
//                }
                foundDevices.add(device)
                adapter.notifyItemInserted(foundDevices.size - 1)
            }
        }))

        // Adding the actions the manager must do before and after scanning
        bleScanManager.beforeScanActions.add { btnStartScan.isEnabled = false }
        bleScanManager.beforeScanActions.add {
            foundDevices.size.let {
                foundDevices.clear()
                adapter.notifyItemRangeRemoved(0, it)
            }
        }
        bleScanManager.afterScanActions.add { btnStartScan.isEnabled = true }

        // Adding the onclick listener to the start scan button
        btnStartScan = findViewById(R.id.btn_start_calibration)
        plotView1 = findViewById(R.id.plotView1)
        plotView2 = findViewById(R.id.plotView2)
        btnStartCalibration = findViewById(R.id.btn_start_calculate)
//        progressBar = findViewById(R.id.progressBar)
//        startButton = findViewById(R.id.startButton)
//        statusText = findViewById(R.id.statusText)
        btnStartCalibration.setOnClickListener{
            val fileName = "debug.csv"

            val externalStorageDirectory = Environment.getExternalStoragePublicDirectory(
                Environment.DIRECTORY_DOCUMENTS);
            val inputFile = File(externalStorageDirectory, fileName)
            Log.w("File_position", "Sensor $inputFile")
            try {
                val fileReader = FileReader(inputFile)
                val bufferedReader = BufferedReader(fileReader)
                var line: String?

                // Read the calibration file
                while (bufferedReader.readLine().also { line = it } != null) {

//                    val data = line!!.split(",").map { it.trim().toDouble() }.toDoubleArray()
//                    val size = data.size
//                    Log.w("data_size", "Sensor $size")
//                    if(data.size==24) {
//                        CalibrationData.add(data)
//                    }
//                    Log.w("Read_csv", "Sensor $line")
                    val data = parseLine(line)
                    val size = data.size
                    Log.w("data_size", "Sensor $size")
                    if(data.size==30) {
                        CalibrationData.add(data)
                    }
                    Log.w("Read_csv", "Sensor $line")
                    println(line)
                }
                val calibration = Calibration(nSensor = 10, readings = CalibrationData)

               //Obtain the calibration results
                val (offset, scale) = calibration.caliResult()
                calibrationOffset = offset
                calibrationScale = scale
                isCalibrated = true // Indicate that calibration has been completed
                printArrayToLog("CalibrationResult", "Offset", offset)
                printArrayToLog("CalibrationResult", "Scale", scale)
                bufferedReader.close()
                fileReader.close()
            } catch (e: IOException) {
                e.printStackTrace()
                println("Error reading file: ${e.message}")
            }
        }
//
        btnStartScan.setOnClickListener{
                permissionManager checkRequestAndDispatch BLE_PERMISSION_REQUEST_CODE
                progressStatus = 0
//                progressBar.progress = progressStatus
//                statusText.text = ""
//                startProgress()
                disconnectBleConnection()
        }
//        progressBar = findViewById(R.id.progressBar)
//        btnStartScan.setOnClickListener {
//            // if (DEBUG) Log.i(TAG, "${it.javaClass.simpleName}:${it.id} - onClick event")
//
//            // Checks if the required permissions are granted and starts the scan if so, otherwise it requests them
//            permissionManager checkRequestAndDispatch BLE_PERMISSION_REQUEST_CODE
//
//        }
    }

    private fun updatePlot(subsetArray: DoubleArray) {
        val x1 = subsetArray[0].toFloat() / 10 // Scale for better visibility
        val y1 = subsetArray[1].toFloat() / 10
        val y2 = subsetArray[2].toFloat() / 10

//        val x1 = subsetArray[0].toFloat()
//        val y1 = subsetArray[1].toFloat()
//        val y2 = subsetArray[2].toFloat()

        val pointSize = 10f // Set point size
        plotView1.addPoint(x1, y1, pointSize)
        plotView2.addPoint(y2, y1, pointSize)
    }
//    private fun updatePlot(subsetArray: DoubleArray) {
//        val x1 = subsetArray[0].toFloat() / 10 // Scale for better visibility
//        val y1 = subsetArray[1].toFloat() / 10
//        val y2 = subsetArray[2].toFloat() / 10
//
//        plotView1.addPoint(x1, y1)
//        plotView2.addPoint(y1, y2)
//    }
    private fun updateResults(subsetArray: DoubleArray) {
        val formattedResults = subsetArray.joinToString(", ") { "%.2f".format(it) }
        val newText = "(x,y,z,theta,phi): ($formattedResults)"
        resultsTextView.text = newText
    }
//    private fun updateResults(results: DoubleArray) {
//        val formattedResults = results.joinToString(", ") { "%.2f".format(it) }
//        val newText = "New Result: [$formattedResults]"
//        resultsAdapter.addResult(newText)
//
//        // Scroll to the latest item
//        resultsRecyclerView.scrollToPosition(resultsAdapter.itemCount - 1)
//    }
    fun printArrayToLog(tag: String, message: String, array: Array<DoubleArray>) {
        Log.d(tag, "$message:")
        array.forEach { subArray ->
            Log.d(tag, subArray.contentToString())
        }
    }
    fun parseLine(line: String?): DoubleArray {
        return try {

            line?.split(",")?.map { it.trim().toDouble() }?.toDoubleArray() ?: DoubleArray(0)
        } catch (e: NumberFormatException) {

            println("Error parsing line: $e")
            DoubleArray(0)
        }
    }

//    private fun startProgress() {
//        val runnable = object : Runnable {
//            override fun run() {
//                if (progressStatus < 100) {
//                    progressStatus++
//                    progressBar.progress = progressStatus
//                    handler.postDelayed(this, 200) // Update the progress every 200 milliseconds
//                } else {
//                    statusText.text = "Calibrated！"
//                }
//            }
//        }
//        handler.post(runnable)
//    }
    private fun disconnectBleConnection() {
        // check the permission
        if (ActivityCompat.checkSelfPermission(
                this,
                Manifest.permission.BLUETOOTH_CONNECT
            ) != PackageManager.PERMISSION_GRANTED
        ) {

            ActivityCompat.requestPermissions(
                this,
                arrayOf(Manifest.permission.BLUETOOTH_CONNECT),
                REQUEST_BLUETOOTH_CONNECT_PERMISSION
            )
            return
        }


        bluetoothGatt?.let { gatt ->
            gatt.disconnect()
            gatt.close()
            bluetoothGatt = null
            Log.w("BluetoothGattCallback", "Disconnected and GATT connection closed")
            runOnUiThread {
                Toast.makeText(this, "BLE Connection was disconnected", Toast.LENGTH_SHORT).show()
            }
        } ?: run {
            Log.w("BluetoothGattCallback", "BluetoothGatt not initialized or already disconnected")
        }
    }




    /**
     * Function that checks whether the permission was granted or not
     */
    @RequiresApi(Build.VERSION_CODES.S)
    override fun onRequestPermissionsResult(
        requestCode: Int, permissions: Array<out String>, grantResults: IntArray
    ) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        permissionManager.dispatchOnRequestPermissionsResult(requestCode, grantResults)
    }

    companion object {
        init {
            System.loadLibrary("mynativeso")
        }
        external fun solve_1mag( readings1:DoubleArray,
                                 psensor1:DoubleArray,
                                 init_param1:DoubleArray): DoubleArray
        private val TAG = CalibrationActivity::class.java.simpleName

        private const val BLE_PERMISSION_REQUEST_CODE = 1
        private const val REQUEST_BLUETOOTH_CONNECT_PERMISSION = 1
        @RequiresApi(Build.VERSION_CODES.S)
        private val blePermissions = arrayOf(
            Manifest.permission.BLUETOOTH_SCAN,
            Manifest.permission.ACCESS_COARSE_LOCATION,
            Manifest.permission.ACCESS_FINE_LOCATION,
            Manifest.permission.BLUETOOTH_ADMIN,
            Manifest.permission.BLUETOOTH,
            Manifest.permission.BLUETOOTH_CONNECT
        )
    }

}


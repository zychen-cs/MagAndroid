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
                // 正确的方法：调用 Double 类型的 pow 方法
                result[r, c] = this[r, c].pow(power.toDouble())
            }
        }
        return result
    }

}

//class CalibrationActivity : AppCompatActivity() {
//    override fun onCreate(savedInstanceState: Bundle?) {
//        super.onCreate(savedInstanceState)
//        setContentView(R.layout.activity_calibration)
//    }
//}

class CalibrationActivity : AppCompatActivity() {
    //    private val WRITE_EXTERNAL_STORAGE_REQUEST_CODE = 123
//    override fun onCreate(savedInstanceState: Bundle?) {
//        super.onCreate(savedInstanceState)
//        setContentView(R.layout.activity_main)
//
//        // 检查权限
//        if (ContextCompat.checkSelfPermission(this, Manifest.permission.WRITE_EXTERNAL_STORAGE)
//            != PackageManager.PERMISSION_GRANTED
//        ) {
//            // 请求权限
//            ActivityCompat.requestPermissions(
//                this,
//                arrayOf(Manifest.permission.WRITE_EXTERNAL_STORAGE),
//                WRITE_EXTERNAL_STORAGE_REQUEST_CODE
//            )
//        } else {
//            // 权限已授予，保存文件
//            saveToFile()
//        }
//    }
//    override fun onRequestPermissionsResult(
//        requestCode: Int,
//        permissions: Array<out String>,
//        grantResults: IntArray
//    ) {
//        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
//        if (requestCode == WRITE_EXTERNAL_STORAGE_REQUEST_CODE) {
//            if (grantResults.isNotEmpty() && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
//                // 权限已授予，保存文件
//                saveToFile()
//            }
//        }
//    }
//    private fun saveToFile() {
//        val data = "Your CSV data here"
//        val fileName = "debug.csv"
//
//        // 获取外部存储目录
//        val externalFilesDir = getExternalFilesDir(Environment.DIRECTORY_DOCUMENTS)
//
//        // 创建文件
//        val file = File(externalFilesDir, fileName)
//
//        // 写入数据
//        try {
//            FileOutputStream(file).use { outputStream ->
//                outputStream.write(data.toByteArray())
//            }
//        } catch (e: Exception) {
//            e.printStackTrace()
//        }
//    }
//    [3, -3, -1.63],
//    [-3, -3, -1.63],
//
//    [-3, 3, -1.63],
//    [3, 3, -1.63],
//    [0, -3, 1.63],
//    [-3, 0, 1.63],
//    [3, 0, 1.63],
//    [0, 3, 1.63],
    var psensor=doubleArrayOf(3.0, -3.0, -1.63,-3.0, -3.0, -1.63,-3.0, 3.0, -1.63
        ,3.0, 3.0, -1.63,3.0, 0.0, 1.63,0.0, -3.0, 1.63,-3.0, 0.0, 1.63,0.0, 3.0, 1.63)

    var initparam = doubleArrayOf(40 / sqrt(2.0) * 1e-6,40 / sqrt(2.0) * 1e-6,0.0,
        ln(0.2),(0.04),(0.02),(-0.0017),Math.PI,0.0)
    var initparam1 = doubleArrayOf(40 / sqrt(2.0) * 1e-6,40 / sqrt(2.0) * 1e-6,0.0,
        ln(0.2),(0.04),(0.02),(-0.0017),Math.PI,0.0)

    val name = listOf(
        "Sensor 1x","Sensor 1y","Sensor 1z","Sensor 2x", "Sensor 2y","Sensor 2z", "Sensor 3x","Sensor 3y","Sensor 3z",
        "Sensor 4x","Sensor 4y","Sensor 4z", "Sensor 5x","Sensor 5y","Sensor 5z", "Sensor 6x", "Sensor 6y","Sensor 6z",
        "Sensor 7x", "Sensor 7y","Sensor 7z","Sensor 8x","Sensor 8y","Sensor 8z"
    )
    var scale = doubleArrayOf(27.4021855,  30.27834967, 30.74578898,
        21.0553706,  21.56542682, 22.10253447,
        27.58085615, 29.98936592, 30.41150167,
        28.19789787, 30.55525519, 31.63808044,
        28.93276359, 31.02736453, 31.75919481,
        27.61959075, 29.82476367 ,30.26779162,
        27.64972945 ,29.83675039, 30.00899469,
        28.75922528, 30.67937347 ,31.97599945)
    val readingsList = mutableListOf<DoubleArray>()
    val CalibrationData = mutableListOf<DoubleArray>()
    var offset = doubleArrayOf(
        24.00578237, -33.38052132, 50.00232938,
        9.89232145, -17.61125348,  48.00466119,
        29.46347733,   9.54043998,  78.61956126,
        28.88195761,  -8.8209807,   87.6507064 ,
        27.63188379, -18.45077436 , 86.0688332 ,
        -2.66079878, -46.29495951,  47.32361657,
        10.42447364, -23.52916744,  60.59391064,
        -1.04093372, -43.9648895,   38.37410865)
    var myparam:DoubleArray = DoubleArray(9)
    private lateinit var btnStartScan: Button
    private lateinit var resultsRecyclerView: RecyclerView
    private lateinit var resultsAdapter: ResultsAdapter
    private lateinit var resultsTextView: TextView
    private lateinit var resultsScrollView: ScrollView
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

            var readings:DoubleArray = DoubleArray(24)

            val currentDateTime = LocalDateTime.now()

            // 格式化当前时间
            val formatter = DateTimeFormatter.ofPattern("yyyy-MM-dd HH:mm:ss")
            val formattedDateTime = currentDateTime.format(formatter)
//            val mixedArray: Array<Any> = arrayOf(formattedDateTime, readings)
//            readings[0]=formattedDateTime
            for (i in 0..7) {
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
            readingsList.add(readings)
            //下面代码实现读取手机文件数据
//            val fileName = "debug.csv"
////            val dataDirectory: File = Environment.getExternalStoragePublicDirectory(
//////                Environment.DIRECTORY_DOCUMENTS);
//            val externalStorageDirectory = Environment.getExternalStoragePublicDirectory(
//                Environment.DIRECTORY_DOCUMENTS);
//            val inputFile = File(externalStorageDirectory, fileName)
//
//            try {
//                val fileReader = FileReader(inputFile)
//                val bufferedReader = BufferedReader(fileReader)
//                var line: String?
//
//                // 逐行读取文件内容
//                while (bufferedReader.readLine().also { line = it } != null) {
//                    // 在这里处理读取到的每一行数据
//                    Log.w("Read_csv", "Sensor $line")
//                    println(line)
//                }
//
//                bufferedReader.close()
//                fileReader.close()
//            } catch (e: IOException) {
//                e.printStackTrace()
//                println("Error reading file: ${e.message}")
//            }

//            下面代码实现写入手机数据
//            val fileName = "debug.csv"
//            val dataDirectory: File = Environment.getExternalStoragePublicDirectory(
//                Environment.DIRECTORY_DOCUMENTS);
//            val outputFile = File(dataDirectory, fileName)
//
//            try {
//                outputFile.printWriter().use { out ->
//                    out.println(name.joinToString(","))
//                    readingsList.forEach { row ->
//                        out.println(row.joinToString(","))
//                    }
//                }
//                println("File saved successfully at ${outputFile.absolutePath}")
//            } catch (e: Exception) {
//                e.printStackTrace()
//                println("Error saving file: ${e.message}")
//            }
            for (i in psensor.indices) {
                psensor[i] *= 1e-2
            }
            if (isCalibrated) {
                // Calculate the mean of all scales to use as the normalization factor
                var totalScaleSum = 0.0
                var count = 0

                // Sum up all individual scale factors across each sensor's three axes
                for (scale in calibrationScale) {
                    totalScaleSum += scale.sum()
                    count += scale.size
                }

                // Calculate the mean scale
                val meanScale = totalScaleSum / count

                // Apply calibration offset and scale, then normalize using the mean scale
                for (i in 0..7) {
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
                Log.w("init results", "results ${result[0]},${result[1]},${result[2]},${result[3]},${result[4]},${result[5]}," +
                        "${result[6]},${result[7]},${result[8]}")
            } else {
                Log.w("BluetoothGattCallback", "No calibration data available.")
            }
//           scale =
//            var sum:Double= 0.0
//
//            for (i in 0..23){
//                sum = sum + scale[i]
//
//            }
//            for (i in psensor.indices) {
//                psensor[i] *= 1e-2
//            }
//            Log.w("sum", "results ${sum}")
//            for (i in 0..23){
//                readings[i] =(readings[i]-offset[i])/scale[i]*(sum/24)
//
//            }
            Runtime.getRuntime().addShutdownHook(Thread {
                println("Output csv")
                val outputFile = File("debug.csv")
                outputFile.printWriter().use { out ->
                    out.println(name.joinToString(","))
                    readingsList.forEach { row ->
                        out.println(row.joinToString(","))
                    }
                    println("Exited")
                }
            })

//
//            // 调用solve_1mag并传入更新后的init
////            val result: DoubleArray = solve_1mag(readings, psensor, initparam)
//

//
//
//
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
                            BluetoothGattDescriptor.ENABLE_INDICATION_VALUE
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

        resultsRecyclerView = findViewById(R.id.resultsRecyclerView)
        resultsRecyclerView.layoutManager = LinearLayoutManager(this)

        // Set up the adapter
        resultsAdapter = ResultsAdapter()
        resultsRecyclerView.adapter = resultsAdapter
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

            // test the nrf52832
            if (it?.device?.address == "FA:0A:21:CD:68:61") {
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

                // 逐行读取文件内容
                while (bufferedReader.readLine().also { line = it } != null) {
                    // 在这里处理读取到的每一行数据
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
                    if(data.size==24) {
                        CalibrationData.add(data)
                    }
                    Log.w("Read_csv", "Sensor $line")
                    println(line)
                }
                val calibration = Calibration(nSensor = 8, readings = CalibrationData)

                // 调用 caliResult 方法来获取校准结果
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
//        btnStartCalibration.setOnClickListener{
//            val fileName = "debug.csv"
////            val dataDirectory: File = Environment.getExternalStoragePublicDirectory(
//////                Environment.DIRECTORY_DOCUMENTS);
//            val externalStorageDirectory = Environment.getExternalStoragePublicDirectory(
//                Environment.DIRECTORY_DOCUMENTS);
//            val inputFile = File(externalStorageDirectory, fileName)
//
//            try {
//                val fileReader = FileReader(inputFile)
//                val bufferedReader = BufferedReader(fileReader)
//                var line: String?
//
//                // 逐行读取文件内容
//                while (bufferedReader.readLine().also { line = it } != null) {
//                    // 在这里处理读取到的每一行数据
//                    val data = line!!.split(",").map { it.trim().toDouble() }.toDoubleArray()
//                    val size = data.size
//                    Log.w("data_size", "Sensor $size")
//                    if(data.size==24) {
//                        CalibrationData.add(data)
//                    }
//                    Log.w("Read_csv", "Sensor $line")
//
//                }
////                val calibration = Calibration(nSensor = 8, readings = CalibrationData)
////
////                // 调用 caliResult 方法来获取校准结果
////                val (offset, scale) = calibration.caliResult()
////                Log.w("offset", "offset $offset")
////                Log.w("scale", "scale $scale")
////                caliResult(calibrationData)
//                bufferedReader.close()
//                fileReader.close()
//            } catch (e: IOException) {
//                e.printStackTrace()
//                println("Error reading file: ${e.message}")
//            }
//        }
        btnStartScan.setOnClickListener{
                permissionManager checkRequestAndDispatch BLE_PERMISSION_REQUEST_CODE
                progressStatus = 0
//                progressBar.progress = progressStatus
//                statusText.text = ""
//                startProgress()
                disconnectBleConnection()  // 断开BLE连接
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
    private fun updateResults(results: DoubleArray) {
        val formattedResults = results.joinToString(", ") { "%.2f".format(it) }
        val newText = "New Result: [$formattedResults]"
        resultsAdapter.addResult(newText)

        // Scroll to the latest item
        resultsRecyclerView.scrollToPosition(resultsAdapter.itemCount - 1)
    }
    fun printArrayToLog(tag: String, message: String, array: Array<DoubleArray>) {
        Log.d(tag, "$message:")
        array.forEach { subArray ->
            Log.d(tag, subArray.contentToString())
        }
    }
    fun parseLine(line: String?): DoubleArray {
        return try {
            // 尝试分割字符串并转换为 DoubleArray
            line?.split(",")?.map { it.trim().toDouble() }?.toDoubleArray() ?: DoubleArray(0)
        } catch (e: NumberFormatException) {
            // 处理转换错误
            println("Error parsing line: $e")
            DoubleArray(0) // 返回空的 DoubleArray 或其他适当的默认值
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
//                    statusText.text = "校准完成！"
//                }
//            }
//        }
//        handler.post(runnable)
//    }
    private fun disconnectBleConnection() {
        // 检查蓝牙连接权限
        if (ActivityCompat.checkSelfPermission(
                this,
                Manifest.permission.BLUETOOTH_CONNECT
            ) != PackageManager.PERMISSION_GRANTED
        ) {
            // 如果权限未被授予，请求权限
            ActivityCompat.requestPermissions(
                this,
                arrayOf(Manifest.permission.BLUETOOTH_CONNECT),
                REQUEST_BLUETOOTH_CONNECT_PERMISSION
            )
            return
        }

        // 如果权限已被授予，继续断开连接
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


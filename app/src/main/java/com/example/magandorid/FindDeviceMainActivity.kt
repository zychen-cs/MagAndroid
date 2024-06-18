package com.example.magandorid
import android.Manifest
import android.annotation.SuppressLint
import android.bluetooth.BluetoothGatt
import android.bluetooth.BluetoothGattCallback
import android.bluetooth.BluetoothGattCharacteristic
import android.bluetooth.BluetoothGattDescriptor
import android.bluetooth.BluetoothManager
import android.bluetooth.BluetoothProfile
import android.content.Intent
import android.content.pm.PackageManager
import android.net.Uri
import android.os.Build
import android.os.Bundle
import android.os.Environment
import android.os.Handler
import android.os.Looper
import android.util.Log
import android.view.View
import android.widget.Button
import android.widget.ProgressBar
import android.widget.TextView
import android.widget.Toast
import androidx.annotation.RequiresApi
import androidx.appcompat.app.AlertDialog
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
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
import android.provider.Settings
import java.io.FileNotFoundException

class FindDeviceMainActivity : AppCompatActivity() {

    //Magx File
//    val name = listOf(
//        "Sensor 1x","Sensor 1y","Sensor 1z","Sensor 2x", "Sensor 2y","Sensor 2z", "Sensor 3x","Sensor 3y","Sensor 3z",
//        "Sensor 4x","Sensor 4y","Sensor 4z", "Sensor 5x","Sensor 5y","Sensor 5z", "Sensor 6x", "Sensor 6y","Sensor 6z",
//        "Sensor 7x", "Sensor 7y","Sensor 7z","Sensor 8x","Sensor 8y","Sensor 8z"
//    )

    //MagDot File
    val name = listOf(
        "Sensor 1x","Sensor 1y","Sensor 1z","Sensor 2x", "Sensor 2y","Sensor 2z", "Sensor 3x","Sensor 3y","Sensor 3z",
        "Sensor 4x","Sensor 4y","Sensor 4z", "Sensor 5x","Sensor 5y","Sensor 5z", "Sensor 6x", "Sensor 6y","Sensor 6z",
        "Sensor 7x", "Sensor 7y","Sensor 7z","Sensor 8x","Sensor 8y","Sensor 8z","Sensor 9x","Sensor 9y","Sensor 9z",
        "Sensor 10x","Sensor 10y","Sensor 10z"
    )
    val CalibrationData = mutableListOf<DoubleArray>()

    val readingsList = mutableListOf<DoubleArray>()

    var myparam:DoubleArray = DoubleArray(9)
    private lateinit var calibrationOffset: Array<DoubleArray>
    private lateinit var calibrationScale: Array<DoubleArray>
    private var isCalibrated = false
    private lateinit var btnStartScan: Button
    private lateinit var btnStartCalibration: Button
    private lateinit var progressBar: ProgressBar
    private val handler = Handler()
    private val PERMISSION_REQUEST_CODE = 100
    private lateinit var statusText: TextView
    private lateinit var resultsTextView: TextView
    private lateinit var permissionManager: PermissionManager
    private lateinit var btManager: BluetoothManager
    private lateinit var bleScanManager: BleScanManager
    private var progressStatus = 0
    private lateinit var foundDevices: MutableList<BleDevice>
    // Top level declaration
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

//            var readings:DoubleArray = DoubleArray(24)
            var readings:DoubleArray = DoubleArray(30)
            var alertCalibrationNeeded = false





            for (i in 0..9) {
                    val value1 = buffer.getFloat(12 * i)
                    val value2 = buffer.getFloat(12 * i + 4)
                    val value3 = buffer.getFloat(12 * i + 8)
                    var j = i + 1

                    readings[3*i]=value1.toDouble()
                    readings[3*i+1]=value2.toDouble()
                    readings[3*i+2]=value3.toDouble()
                    if (Math.abs(value1) > 200 || Math.abs(value2) > 200 || Math.abs(value3) > 200) {
                    alertCalibrationNeeded = true
                }
//                    temp = readings.
//                    Log.w("readings", "Sensor ($readings.get(0),$readings.get(1),$readings.get(2))")
                    Log.w("BluetoothGattCallback", "Sensor $j: ($value1, $value2, $value3)")
            }
            readingsList.add(readings)
            if (alertCalibrationNeeded) {
                // Show an AlertDialog on the main UI thread
                Handler(Looper.getMainLooper()).post {
                    if (ActivityCompat.checkSelfPermission(
                            this@FindDeviceMainActivity,
                            Manifest.permission.BLUETOOTH_CONNECT
                        ) != PackageManager.PERMISSION_GRANTED
                    ) {
                        // Request permission if not granted
                        ActivityCompat.requestPermissions(
                            this@FindDeviceMainActivity,
                            arrayOf(Manifest.permission.BLUETOOTH_CONNECT),
                            BLE_PERMISSION_REQUEST_CODE
                        )
                        return@post
                    }

                    // Disconnect if permission is granted
                    gatt.disconnect()
                    gatt.close()

//                    gatt.
                    val alertDialogBuilder = AlertDialog.Builder(this@FindDeviceMainActivity)
                    alertDialogBuilder.setTitle("Calibration Warning")
                    alertDialogBuilder.setMessage("Warning: Sensor readings exceed calibration limits. Please recalibrate.")
                    alertDialogBuilder.setPositiveButton("OK") { dialog, _ ->
                        dialog.dismiss()
                    }
                    alertDialogBuilder.setCancelable(false)
                    alertDialogBuilder.show()
                }
            }

            val fileName = "debug.csv"
            val dataDirectory: File = Environment.getExternalStoragePublicDirectory(
                Environment.DIRECTORY_DOCUMENTS);
            val outputFile = File(dataDirectory, fileName)

            try {
                outputFile.printWriter().use { out ->
                    out.println(name.joinToString(","))
                    readingsList.forEach { row ->
                        out.println(row.joinToString(","))
                    }
                }
                println("File saved successfully at ${outputFile.absolutePath}")
            } catch (e: Exception) {
                e.printStackTrace()
                println("Error saving file: ${e.message}")
            }

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


//
                // Enable notifications
//                if (descriptor != null) {
//                    CoroutineScope(Dispatchers.IO).launch {
//                        descriptor.apply {
//                            value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
//                            gatt.writeDescriptor(this)
//                        }
//                    }
//                }


//                if (descriptor != null) {
//                    descriptor?.apply {
//                        value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
//                        gatt.writeDescriptor(this)
//                    }
//                }
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
        setContentView(R.layout.activity_find_device_main)
        //初始化权限
        permissionManager = PermissionManager(this)
        permissionManager buildRequestResultsDispatcher {
            withRequestCode(BLE_PERMISSION_REQUEST_CODE) {
                checkPermissions(blePermissions)
                showRationaleDialog(getString(R.string.ble_permission_rationale))
                doOnGranted { bleScanManager.scanBleDevices() }
                doOnDenied {
                    Toast.makeText(
                        this@FindDeviceMainActivity,
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
            if (it?.device?.address == "C3:A3:6A:87:35:84") {
                Log.w("ScanResultAdapter", "Connecting to $address")
                val gatt = it?.device?.connectGatt(this, false, gattCallback)
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
        btnStartScan = findViewById(R.id.btn_start_scan)
        btnStartCalibration = findViewById(R.id.btn_start_calculate)
        progressBar = findViewById(R.id.progressBar)
        statusText = findViewById(R.id.statusText)
        resultsTextView = findViewById(R.id.resultsTextView)
        btnStartCalibration.setOnClickListener{
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.R) {
                if (!Environment.isExternalStorageManager()) {
                    try {
                        val intent = Intent(Settings.ACTION_MANAGE_APP_ALL_FILES_ACCESS_PERMISSION)
                        val uri = Uri.fromParts("package", packageName, null)
                        intent.data = uri
                        startActivity(intent)
                    } catch (ex: Exception) {
                        val intent = Intent(Settings.ACTION_MANAGE_ALL_FILES_ACCESS_PERMISSION)
                        startActivity(intent)
                    }
                }
            } else {
                if (ContextCompat.checkSelfPermission(this, Manifest.permission.WRITE_EXTERNAL_STORAGE) != PackageManager.PERMISSION_GRANTED) {
                    ActivityCompat.requestPermissions(this, arrayOf(Manifest.permission.WRITE_EXTERNAL_STORAGE, Manifest.permission.READ_EXTERNAL_STORAGE), PERMISSION_REQUEST_CODE)
                }
            }

            // Your file reading logic here
            readFile()

        }

        btnStartScan.setOnClickListener {
            // if (DEBUG) Log.i(TAG, "${it.javaClass.simpleName}:${it.id} - onClick event")

            // Checks if the required permissions are granted and starts the scan if so, otherwise it requests them
            permissionManager checkRequestAndDispatch BLE_PERMISSION_REQUEST_CODE
            progressStatus = 0
            progressBar.progress = progressStatus
            statusText.text = ""
            startProgress()
        }
    }
    private fun readFile() {
        //            val fileName = "debug.csv"
//            val fileName = "debug.csv"
//            val dataDirectory: File? = getExternalFilesDir(Environment.DIRECTORY_DOCUMENTS)
//            val inputFile = File(dataDirectory, fileName)
        val fileName = "debug.csv"
//        val dataDirectory: File? = getExternalFilesDir(Environment.DIRECTORY_DOCUMENTS)
//        val inputFile = File(dataDirectory, fileName)

// Writing to outputFile can be done without any permissions

            val externalStorageDirectory = Environment.getExternalStoragePublicDirectory(
                Environment.DIRECTORY_DOCUMENTS);
            val inputFile = File(externalStorageDirectory, fileName)
        Log.w("File_position", "Sensor $inputFile")
        try {
            val fileReader = FileReader(inputFile)
            val bufferedReader = BufferedReader(fileReader)
            var line: String?


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

            // Obtain calibration results
            val (offset, scale) = calibration.caliResult()
            calibrationOffset = offset
            calibrationScale = scale
            resultsTextView.text = formatDoubleArray(calibrationScale)
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

    override fun onRequestPermissionsResult(requestCode: Int, permissions: Array<out String>, grantResults: IntArray) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        if (requestCode == PERMISSION_REQUEST_CODE) {
            if ((grantResults.isNotEmpty() && grantResults[0] == PackageManager.PERMISSION_GRANTED)) {

                readFile()
            } else {

                Log.w("Permission", "Permission denied to read/write external storage")
            }
        }
    }


    private fun formatDoubleArray(array: Array<DoubleArray>): String {
        return array.joinToString(separator = "\n") { subArray ->
            subArray.joinToString(separator = ", ") { "%.2f".format(it) }
        }
    }
    fun printArrayToLog(tag: String, message: String, array: Array<DoubleArray>) {
        Log.d(tag, "$message:")
        array.forEach { subArray ->
            Log.d(tag, subArray.contentToString())
        }
    }

    fun parseLine(line: String?): DoubleArray {
        return try {
            // Spilt the String
            line?.split(",")?.map { it.trim().toDouble() }?.toDoubleArray() ?: DoubleArray(0)
        } catch (e: NumberFormatException) {
            // handle the error
            println("Error parsing line: $e")
            DoubleArray(0)
        }
    }
    private fun startProgress() {
        val runnable = object : Runnable {
            override fun run() {
                if (progressStatus < 100) {
                    progressStatus++
                    progressBar.progress = progressStatus
                    handler.postDelayed(this, 200) // Update the progress every 200 milliseconds
                } else {
                    statusText.text = "Calibration completed!"
                }
            }
        }
        handler.post(runnable)
    }
    /**
     * Function that checks whether the permission was granted or not
     */



//    @RequiresApi(Build.VERSION_CODES.S)
//    override fun onRequestPermissionsResult(
//        requestCode: Int, permissions: Array<out String>, grantResults: IntArray
//    ) {
//        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
//        permissionManager.dispatchOnRequestPermissionsResult(requestCode, grantResults)
//    }

    companion object {
        init {
            System.loadLibrary("mynativeso")
        }
        external fun solve_1mag( readings1:DoubleArray,
                                 psensor1:DoubleArray,
                                 init_param1:DoubleArray): DoubleArray
        private val TAG = FindDeviceMainActivity::class.java.simpleName

        private const val BLE_PERMISSION_REQUEST_CODE = 1
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


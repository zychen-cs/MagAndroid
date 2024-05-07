package com.example.magandorid

import android.content.Intent
import androidx.appcompat.app.AppCompatActivity
import android.os.Bundle
import com.example.magandorid.databinding.ActivityRealTimeBinding
import com.example.magandorid.databinding.ActivityStartBinding

class RealTimeActivity : AppCompatActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_real_time)

        val binding = ActivityRealTimeBinding.inflate(layoutInflater)
        setContentView(binding.root)

        binding.btnStartReading.setOnClickListener {
            val intent = Intent(this, ConfigurationActivity::class.java)
            startActivity(intent)
        }
    }
}
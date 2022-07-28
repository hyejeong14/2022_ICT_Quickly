package com.example.test12

import android.content.Intent
import android.os.Bundle
import android.view.MenuItem
import androidx.appcompat.app.AppCompatActivity
import kotlinx.android.synthetic.main.callhistory.*
import kotlinx.android.synthetic.main.race.*

class RaceActivity : AppCompatActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.race)

        setSupportActionBar(race_toolbar)
        supportActionBar?.setDisplayHomeAsUpEnabled(true)
    }

    override fun onOptionsItemSelected(item: MenuItem): Boolean {
        val myintent = Intent(this,MainActivity::class.java)
        startActivity(myintent)
        return true
    }
}
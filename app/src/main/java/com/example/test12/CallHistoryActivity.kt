package com.example.test12

import android.content.Intent
import android.os.Bundle
import android.view.Menu
import android.view.MenuItem
import android.widget.Button
import android.widget.Toolbar
import androidx.appcompat.app.AppCompatActivity
import androidx.recyclerview.widget.LinearLayoutManager
import kotlinx.android.synthetic.main.callhistory.*
import androidx.appcompat.app.ActionBar as ActionBar

class CallHistoryActivity : AppCompatActivity() {

    val DataList = arrayListOf(
        Data(R.drawable.siren,"0번"),
        Data(R.drawable.siren,"1번"),
        Data(R.drawable.siren,"2번"),
        Data(R.drawable.siren,"3번"),
        Data(R.drawable.siren,"4번"),
        Data(R.drawable.siren,"5번"),
        Data(R.drawable.siren,"6번"),
        Data(R.drawable.siren,"7번"),
        Data(R.drawable.siren,"8번"),
        Data(R.drawable.siren,"9번"),
        Data(R.drawable.siren,"10번"),
        Data(R.drawable.siren,"11번")
    )
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.callhistory)

        setSupportActionBar(callhistory_toolbar)
        supportActionBar?.setDisplayHomeAsUpEnabled(true)

        call_recyclerView.layoutManager = LinearLayoutManager(this)
        call_recyclerView.adapter = CustomAdapter(DataList)
    }

    override fun onOptionsItemSelected(item: MenuItem): Boolean {
        val myintent = Intent(this,MainActivity::class.java)
        startActivity(myintent)
        return true
    }
}
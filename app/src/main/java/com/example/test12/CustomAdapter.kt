package com.example.test12

import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.Toast
import androidx.recyclerview.widget.RecyclerView
import kotlinx.android.synthetic.main.call_list.view.*

class Data(val profile:Int, val name:String)

class CustomViewHolder(v : View) : RecyclerView.ViewHolder(v){
    val profile = v.call_image
    val name = v.call_text
}

class CustomAdapter(val DataList:ArrayList<Data>) : RecyclerView.Adapter<CustomViewHolder>(){
    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): CustomViewHolder {
        val cellForRow = LayoutInflater.from(parent.context).inflate(R.layout.call_list,parent,false)
        return CustomViewHolder(cellForRow)
    }

    override fun onBindViewHolder(holder: CustomViewHolder, position: Int) {
        holder.profile.setImageResource(DataList[position].profile)
        holder.name.text = DataList[position].name

        holder.itemView.setOnClickListener { v->
            Toast.makeText(v.context,DataList[position].name,Toast.LENGTH_SHORT).show()
        }
    }

    override fun getItemCount() = DataList.size

}
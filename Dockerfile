FROM ros:humble-ros-base   
WORKDIR /ros_ws      
RUN apt-get update && apt-get install -y python3-pip libgl1 libglib2.0-0 && rm -rf /var/lib/apt/lists/*
RUN pip3 install cflib
RUN pip3 install opencv-python
COPY entrypoint.sh .       
COPY sensor.py .
COPY controller.py .
COPY vision.py .
RUN chmod +x ./entrypoint.sh   
ENTRYPOINT ["./entrypoint.sh"] 
CMD ["python3", "sensor.py"]   

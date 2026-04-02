FROM ros:humble-ros-base   
WORKDIR /ros_ws      
RUN pip install cflib --break-system-packages
COPY entrypoint.sh .       
COPY sensor.py .
COPY controller.py .
RUN chmod +x ./entrypoint.sh   
ENTRYPOINT ["./entrypoint.sh"] 
CMD ["python3", "sensor.py"]   

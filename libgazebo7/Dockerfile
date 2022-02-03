FROM gazebo:libgazebo7

RUN apt-get update && \
    apt-get install -y make cmake build-essential && \
    rm -rf /var/lib/apt/lists/*

COPY compile.sh .
# RUN mkdir app
# WORKDIR app

# ENTRYPOINT ["sh","-c","cd","build","&&","cmake","..","&&","make"]
CMD ["./compile.sh"]

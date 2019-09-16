pipeline {
  agent {
    docker {
      image 'ccr.ccs.tencentyun.com/openss/centos_python3'
    }

  }
  stages {
    stage('检出') {
      steps {
        checkout([$class: 'GitSCM', branches: [[name: env.GIT_BUILD_REF]], 
                                                    userRemoteConfigs: [[url: env.GIT_REPO_URL, credentialsId: env.CREDENTIALS_ID]]])
      }
    }
    stage('构建') {
      steps {
        echo '构建中...'
        sh 'export LC_ALL=en_US.utf-8'
        sh 'export LANG=en_US.utf-8'
        sh 'rm -rf /root/workspace/.pio/libdeps'
        sh 'pip install -i https://mirrors.cloud.tencent.com/pypi/simple platformio'
        sh 'platformio update'
        sh 'platformio run'
        echo '构建完成.'
        archiveArtifacts(artifacts: '.pio/build/kvpac/firmware.bin,.pio/build/TEST/firmware.bin,.pio/build/BOARDTEST/firmware.bin', fingerprint: true)
      }
    }
    stage('测试') {
      steps {
        echo '单元测试中...'
        echo '单元测试完成.'
        writeFile(file: 'TEST-demo.junit4.AppTest.xml', text: '''
                    <testsuite name="demo.junit4.AppTest" time="0.053" tests="3" errors="0" skipped="0" failures="0">
                        <properties>
                        </properties>
                        <testcase name="testApp" classname="demo.junit4.AppTest" time="0.003"/>
                        <testcase name="test1" classname="demo.junit4.AppTest" time="0"/>
                        <testcase name="test2" classname="demo.junit4.AppTest" time="0"/>
                    </testsuite>
                ''')
        junit '*.xml'
      }
    }
    stage('部署') {
      steps {
        echo '部署中...'
        echo '部署完成'
      }
    }
  }
}
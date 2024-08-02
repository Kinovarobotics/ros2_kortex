pipeline {
    agent { label 'ubuntu' }

    stages {
        stage('Checkout') {
            steps {
                git branch: 'development-moveit-gen3-lite', url: 'https://github.com/your-repo.git'
            }
        }
        stage('Setup Python') {
            steps {
                sh 'sudo apt-get update'
                sh 'sudo apt-get install -y python3.10 python3-pip'
                sh 'python3.10 -m venv venv'
                sh '. venv/bin/activate'
            }
        }
        stage('Install System Hooks') {
            steps {
                sh 'sudo apt-get install -y clang-format-14 cppcheck'
            }
        }
        stage('Run Pre-commit') {
            steps {
                sh 'pip install pre-commit'
                sh 'pre-commit run --all-files --hook-stage manual'
            }
        }
    }
}
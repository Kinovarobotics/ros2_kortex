pipeline {
    agent any

    stages {
        stage('Clone Repository') {
            steps {
                // Cloning the GitHub repository
                git branch: 'main', url: 'https://github.com/Kinovarobotics/ros2_kortex.git'
            }
        }
        stage('Validate YAML') {
            steps {
                // Running YAML linting on the specified YAML file
                sh 'yamllint .github/workflows/ci-format.yml'
            }
        }
    }

    post {
        always {
            echo 'Performing cleanup and final steps...'
            // If you have test results, they will be archived and published here
            archiveArtifacts artifacts: '**/test_results/*.xml', allowEmptyArchive: true
            junit 'test_results/*.xml'
        }
        success {
            echo 'Pipeline completed successfully!'
            // Additional actions on success can be added here
        }
        failure {
            echo 'Pipeline failed! Check the logs for more details.'
            // Additional failure handling can be added here, such as notifications
        }
    }
}

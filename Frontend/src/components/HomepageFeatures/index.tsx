import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  Svg: React.ComponentType<React.ComponentProps<'svg'>>;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'ROS2 Fundamentals',
    Svg: require('@site/static/img/ros2-nodes.svg').default,
    description: (
      <>
        Learn how ROS2 acts as a robot&apos;s nervous system. Master nodes, topics,
        services, and URDF modeling for humanoid robots with Python integration.
      </>
    ),
  },
  {
    title: 'Digital Twin Simulation',
    Svg: require('@site/static/img/digital-twin.svg').default,
    description: (
      <>
        Build virtual replicas of your robot for safe testing. Master physics
        simulation with Gazebo, sensor integration, and Unity visualization.
      </>
    ),
  },
  {
    title: 'AI Learning Assistant',
    Svg: require('@site/static/img/ai-assistant.svg').default,
    description: (
      <>
        Get instant help from our RAG-powered AI assistant. Ask questions about
        any topic and receive contextual answers from the textbook content.
      </>
    ),
  },
];

function Feature({title, Svg, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <Svg className={styles.featureSvg} role="img" />
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
